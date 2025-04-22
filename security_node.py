from rclpy.node import Node
from std_msgs.msg import String

from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from cryptography.hazmat.primitives.hashes import SHA256
import secrets
import hashlib
import base64

class SecurityNode(Node):
    def __init__(self, enable_security = False):
        super().__init__('robot_security_node')
        self.get_logger().info("Initializing Security Node...")

        self.enable_security = enable_security

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

            # Publisher for public key and acknowledgment
            self.public_key_publisher = self.create_publisher(String, '/public_key', 10)
            self.aes_key_ack_publisher = self.create_publisher(String, '/aes_key_ack', 10)

            # Timer to repeatedly publish the public key
            self.public_key_timer = self.create_timer(1, self.publish_public_key)

            # Subscriber for public key and acknowledgment
            self.public_key_subscription = self.create_subscription(String, '/public_key', self.receive_public_key_callback, 10)
            self.aes_ack_subscription = self.create_subscription(String, '/aes_key_ack', self.receive_aes_key_ack_callback, 10)
        else:
            self.get_logger().info("Security is disabled. Running without encryption.")


    def publish_public_key(self):
        # Publish the public key
        msg = String()
        msg.data = self.public_key_bytes.decode('utf-8')  # Convert bytes to string
        self.public_key_publisher.publish(msg)
        self.get_logger().info("Published public key.")
    
    def receive_public_key_callback(self, msg):
        try:
            # Deserialize the peer's public key
            peer_public_key_bytes = msg.data.encode('utf-8')  # Convert string back to bytes
            if peer_public_key_bytes == self.public_key_bytes:
                #self.get_logger().info("Received own public key. Ignoring.")
                return
            self.aes_key = self.exchange_keys(peer_public_key_bytes)
            self.get_logger().info("Shared AES key derived successfully.")

            self.send_aes_key_ack()

            # Stop subscribing to the public key topic
            self.destroy_subscription(self.public_key_subscription)
            self.get_logger().info("Unsubscribed from /public_key topic.")
        except Exception as e:
            self.get_logger().error(f"Failed to derive AES key: {e}")

    def send_aes_key_ack(self):
        # Start a timer to repeatedly publish the acknowledgment
        self.ack_timer = self.create_timer(1, self.publish_aes_key_ack)
        self.get_logger().info("Started acknowledgment publishing timer.")

    def publish_aes_key_ack(self):
        # Publish the AES acknowledgment
        msg = String()
        msg.data = base64.b64encode(self.public_key_bytes).decode('utf-8')
        self.aes_key_ack_publisher.publish(msg)
        #self.get_logger().info("Published acknowledgment to peer.")
    
    def receive_aes_key_ack_callback(self, msg):
        try:
            peer_public_key_bytes = base64.b64decode(msg.data.encode('utf-8'))
            if peer_public_key_bytes != self.public_key_bytes:
                self.public_key_timer.cancel()
                #self.get_logger().info("Peer AES key established. Stopped publishing public key.")
        except Exception as e:
            self.get_logger().error(f"Failed to process acknowledgment: {e}")

    def encrypt_and_gmac(self, message, timestamp):
        """
        Encrypt a message and generate GMAC to ensure confidentiality and integrity
        Args:
            message (str): The command message to generate the MAC for.

        Returns:
            str: The GMAC as a hexadecimal string.
        """
        data = f"{message}:{timestamp}".encode()
        nonce = secrets.token_bytes(12)
        encrypted_data = self.aesgcm.encrypt(nonce, data, None)

        return nonce, encrypted_data
        
    def decrypt_and_verify(self, encrypted_data, nonce):
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
            decrypted_data = self.aesgcm.decrypt(nonce, encrypted_data, None).decode()
            message, _ = decrypted_data.split(':', 1)  # Split by the first colon
            return message
        except Exception as e:
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