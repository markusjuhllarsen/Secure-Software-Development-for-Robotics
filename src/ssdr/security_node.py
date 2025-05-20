from rclpy.node import Node
from ssdr_interfaces.srv import KeyExchange

from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from cryptography.hazmat.primitives.hashes import SHA256
import secrets
import hashlib
import base64

class SecurityNode(Node):
    def __init__(self, name, enable_security = False, is_controller = False):
        super().__init__(name)
        self.get_logger().info("Initializing Security Node.")

        self.enable_security = enable_security
        self.is_controller = is_controller

        """Security setup"""
        if self.enable_security:
            # Generate ECDH keys
            self.private_key = ec.generate_private_key(ec.SECP256R1())
            self.public_key = self.private_key.public_key()
            self.public_key_bytes = self.public_key.public_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PublicFormat.SubjectPublicKeyInfo
            )

            self.aes_key = None
            self.aesgcm = None

            self.node_id = hashlib.sha256(self.public_key_bytes).hexdigest()

            if self.is_controller:
                self.public_key_client = self.create_client(KeyExchange, 'security_node/exchange_public_key')
                self.key_exchange_timer = self.create_timer(1, self.initiate_key_exchange)
            else:
                self.create_service(
                    KeyExchange, '/security_node/exchange_public_key', self.exchange_public_key_callback
                )
        else:
            self.get_logger().info("Security is disabled. Running without encryption.")

    def initiate_key_exchange(self):
        """Initiate the key exchange process with the security node."""
        if not self.public_key_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Public key exchange service not available.")
            return

        request = KeyExchange.Request()
        request.requester_public_key = self.public_key_bytes.decode('utf-8')  # Convert bytes to string

        future = self.public_key_client.call_async(request)
        future.add_done_callback(self.handle_key_exchange_response)

    def handle_key_exchange_response(self, future):
        """Handle the response from the key exchange service. (Client-side)"""
        try:
            response = future.result()
            peer_public_key_bytes = response.responder_public_key.encode('utf-8')  # Convert string back to bytes
            self.aes_key = self.exchange_keys(peer_public_key_bytes)
            self.get_logger().info("Shared AES key derived successfully.")
            # Stop the timer after the key exchange is complete
            self.destroy_timer(self.key_exchange_timer)
        except Exception as e:
            self.get_logger().error(f"Failed to complete key exchange: {e}")

    def exchange_public_key_callback(self, request, response):
        """Handle incoming public key exchange requests. (Server-side)"""
        try:
            peer_public_key_bytes = request.requester_public_key.encode('utf-8')  # Convert string back to bytes
            self.aes_key = self.exchange_keys(peer_public_key_bytes)
            self.get_logger().info("Shared AES key derived successfully.")

            # Respond with this node's public key
            response.responder_public_key  = self.public_key_bytes.decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Failed to process public key exchange: {e}")
            response.responder_public_key = ""
        return response

    def encrypt_and_gmac(self, message, timestamp):
        """
        Encrypt a message and generate GMAC to ensure confidentiality and integrity
        Args:
            message (str): The command message to generate the MAC for.

        Returns:
            str: The GMAC as a hexadecimal string.
        """
        # DO WE NEED TIMESTAMP IF USING NONCE??????????????????
        #
        #
        #
        data = f"{message}:{timestamp}".encode()
        nonce = secrets.token_bytes(12)
        encrypted_data = self.aesgcm.encrypt(nonce, data, None)
        combined = nonce + encrypted_data
        return base64.b64encode(combined).decode('utf-8')
        
    def decrypt_and_verify(self, encrypted_message):
        """
        Decrypt the message and verify GMAC to ensure confidentiality and integrity
        Args:
            encrypted_data (bytes): The encrypted command message.
            nonce (bytes): The nonce used for encryption.

        Returns:
            str: The decrypted message.
        """
        try:
            # Decrypt the data and verify GMAC
            # The GMAC is verified during decryption, so if it fails, an exception will be raised
            combined = base64.b64decode(encrypted_message)
            nonce, encrypted_data = combined[:12], combined[12:]
            decrypted_data = self.aesgcm.decrypt(nonce, encrypted_data, None).decode()
            message, _ = decrypted_data.split(':', 1)  # Split by the first colon
            return message
        except Exception as e:
            self.get_logger().error(f"Decryption failed: {e}")
            return None
        
    def exchange_keys(self, peer_public_key_bytes):
        """
        Perform ECDH key exchange to derive a shared AES key.
        Args:
            private_key (PrivateKey): The private key of the local node.
            peer_public_key_bytes (bytes): The public key of the peer in PEM format.
        """
        try:
            # Deserialize the peer's public key
            peer_public_key = serialization.load_pem_public_key(peer_public_key_bytes)

            # Compute the shared secret
            shared_secret = self.private_key.exchange(ec.ECDH(), peer_public_key)

            # Derive the AES key using HKDF
            aes_key = HKDF(
                algorithm=SHA256(),
                length=32,  # AES-256 key length
                salt=None,
                info=b"handshake data"
            ).derive(shared_secret)

            return aes_key
        except Exception as e:
            return None
    
    def publish_status(self, status_text):
        """
        Publish status message and update the GUI
        """
        # Log the status
        self.get_logger().info(status_text)
        
        # Try to publish to the topic if the publisher exists
        #if hasattr(self, 'status_publisher'):
        #    try:
        #        msg = String()
        #        msg.data = status_text
        #        self.status_publisher.publish(msg)
        #    except Exception as e:
        #        self.get_logger().error(f"Error publishing status: {str(e)}")
        
        # Update the GUI status window
        if hasattr(self, 'gui'):
            self.gui.update_status(status_text)