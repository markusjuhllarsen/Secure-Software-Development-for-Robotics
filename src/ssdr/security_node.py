from rclpy.node import Node
from ssdr_interfaces.srv import KeyExchange
from rclpy.task import Future

from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from cryptography.hazmat.primitives.hashes import SHA256
import secrets
import base64
from threading import Condition

class SecurityNode(Node):
    def __init__(self, name, enable_security = False, is_controller = False):
        super().__init__(name)
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
            self.key_exchange_condition = Condition()
            self.aesgcm = None


            if self.is_controller:
                self.public_key_client = self.create_client(KeyExchange, 'srobot_ecurity_node/exchange_public_key')
                self.key_exchange_timer = self.create_timer(1, self.initiate_key_exchange)
            else:
                self.create_service(
                    KeyExchange, '/robot_security_node/exchange_public_key', self.exchange_public_key_callback
                )

    def initiate_key_exchange(self):
        """Initiate the key exchange process with the security node from the controller side."""
        if not self.public_key_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Public key exchange service not available.")
            return

        request = KeyExchange.Request()
        request.requester_public_key = self.public_key_bytes.decode('utf-8')  # Convert bytes to string

        future = self.public_key_client.call_async(request)
        future.add_done_callback(self.handle_key_exchange_response)

    def handle_key_exchange_response(
            self, 
            future: Future
            ) -> None:
        """Handle the response from the key exchange service. (Client-side)
        args:
            future: The future object containing the response.
        """
        try:
            response = future.result()
            peer_public_key_bytes = response.responder_public_key.encode('utf-8')  # Convert string back to bytes
            self.aes_key = self.exchange_keys(peer_public_key_bytes)
            with self.key_exchange_condition:
                self.key_exchange_condition.notify_all()
            self.destroy_timer(self.key_exchange_timer)
        except Exception as e:
            self.get_logger().error(f"Failed to complete key exchange: {e}")
            raise RuntimeError("Key exchange failed") from e

    def exchange_public_key_callback(
            self, 
            request: KeyExchange.Request, 
            response: KeyExchange.Response
            ) -> KeyExchange.Response:
        """Handle incoming public key exchange requests. (Server-side)
        args:
            request: The incoming request containing the peer's public key.
            response: The response object to fill with the responder's public key.
        returns:
            The response containing the responder's public key.
        """
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

    def encrypt_and_gmac(
            self, 
            message: str, 
            timestamp: str
            ) -> str:
        """
        Encrypt a message and generate GMAC to ensure confidentiality and integrity
        args:
            message: The command message to generate the MAC for.
            timestamp: The timestamp to include in the MAC.
        returns:
            The GMAC as a hexadecimal string.
        """
        data = f"{message}:{timestamp}".encode()
        nonce = secrets.token_bytes(12)
        encrypted_data = self.aesgcm.encrypt(nonce, data, None)
        combined = nonce + encrypted_data
        return base64.b64encode(combined).decode('utf-8')
        
    def decrypt_and_verify(
            self, 
            encrypted_message: str
            ) -> str:
        """
        Decrypt the message and verify GMAC to ensure confidentiality and integrity
        args:
            encrypted_data: The encrypted command message.
        returns:
            The decrypted message.
        """
        try:
            # The GMAC is verified during decryption, so if it fails, an exception will be raised
            combined = base64.b64decode(encrypted_message)
            nonce, encrypted_data = combined[:12], combined[12:] # 12 byte nonce
            decrypted_data = self.aesgcm.decrypt(nonce, encrypted_data, None).decode()
            message, _ = decrypted_data.split(':', 1)  # Split by the first colon, as remaining part is timestamp
            return message
        except Exception as e:
            self.get_logger().error(f"Decryption failed: {e}")
            raise RuntimeError("Decryption failed") from e
        
    def exchange_keys(
            self, 
            peer_public_key_bytes: bytes
            ) -> bytes:
        """
        Perform ECDH key exchange to derive a shared AES key.
        args:
            peer_public_key_bytes: The public key of the peer in PEM format.
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
            self.get_logger().error(f"Key exchange failed: {e}")
            raise RuntimeError("Key exchange failed") from e
    
    def publish_status(
            self, 
            status_text: str
            ) -> None:
        """
        Publish status message and update the GUI
        args:
            status_text: The status message to log and update in the GUI.
        """
        # Log the status
        self.get_logger().info(status_text)        
        # Update the GUI status window
        if hasattr(self, 'gui'):
            self.gui.update_status(status_text)