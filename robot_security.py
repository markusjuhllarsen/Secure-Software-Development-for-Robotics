from utils.setup import setup_ros2_environment

setup_ros2_environment()

import rclpy
from rclpy.node import Node
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization
import base64

from geometry_msgs.msg import Twist  # For publishing to cmd_vel
from std_msgs.msg import String  # For receiving encrypted messages

from utils.security import decrypt_and_verify, exchange_keys

class SecurityNode(Node):
    def __init__(self):
        super().__init__('security_node')
        self.get_logger().info("Initializing Security Node...")

        """Security setup"""
        # Generate ECDH keys
        self.private_key = ec.generate_private_key(ec.SECP256R1())
        self.public_key = self.private_key.public_key()
        
        # Serialize the public key for sharing
        self.public_key_bytes = self.public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        )

        self.aes_key = None
        self.peer_ack_received = False
        self.aesgcm = None

        #Pub/sub setup
        self.publisher_list = []
        self._init_publishers()

        self.public_key_timer = self.create_timer(1, self.publish_public_key)

        self._init_subscribers()

        self.get_logger().info("Security Node initialized and ready.")

    def _init_publishers(self):
        """Initialize all publishers"""
        self.publisher_list.append(self.create_publisher(
                Twist,
                '/cmd_vel_unstamped',  # cmd_vel_unstamped topic
                10 # Queue size
            ))
            
        # Secondary publisher for the differential drive controller
        self.publisher_list.append(self.create_publisher(
            Twist,
            '/diffdrive_controller/cmd_vel',
            10
        ))

        self.public_key_publisher = self.create_publisher(
            String,
            'public_key_robot',  # Topic for sharing public key
            10
        )

        self.aes_key_ack_publisher = self.create_publisher(
            String, 
            '/aes_key_ack', 
            10
        )
    
    def _init_subscribers(self):
        self.create_subscription(
            String,
            'encrypted_topic',
            self.decrypt_and_forward_callback,
            10
        )

        self.public_key_subscription = self.create_subscription(
            String, 
            '/public_key_controller', 
            self.receive_public_key_callback, 
            10
        )

        self.create_subscription(
            String, 
            '/aes_key_ack', 
            self.receive_aes_key_ack_callback, 
            10
        )

    def send_aes_key_ack(self):
        msg = String()
        msg.data = f"{self.public_key_bytes}"
        self.aes_key_ack_publisher.publish(msg)
        self.get_logger().info("Sent acknowledgment to peer.")
    
    def receive_aes_key_ack_callback(self, msg):
        try:
            peer_public_key = msg.data  # Split the ACK message into parts
            if peer_public_key != self.public_key_bytes:
                self.peer_ack_received = True
                self.get_logger().info(f"Received acknowledgment from peer")
        except Exception as e:
            self.get_logger().error(f"Failed to process acknowledgment: {e}")

    def publish_public_key(self):
        if self.peer_ack_received:
            # Stop publishing once the AES key is established
            self.public_key_timer.cancel()
            self.get_logger().info("Peer AES key established. Stopped publishing public key.")
            return

        # Publish the public key
        msg = String()
        msg.data = self.public_key_bytes.decode('utf-8')  # Convert bytes to string
        self.public_key_publisher.publish(msg)
        self.get_logger().info("Published public key.")
    
    def receive_public_key_callback(self, msg):
        try:
            # Deserialize the peer's public key
            peer_public_key_bytes = msg.data.encode('utf-8')  # Convert string back to bytes
            self.aes_key = exchange_keys(self.private_key, peer_public_key_bytes)
            print(self.aes_key)
            self.send_aes_key_ack()
            self.get_logger().info("Shared AES key derived successfully.")

            # Stop subscribing to the public key topic
            self.destroy_subscription(self.public_key_subscription)
            self.get_logger().info("Unsubscribed from /public_key topic.")
        except Exception as e:
            self.get_logger().error(f"Failed to derive AES key: {e}")

    def decrypt_and_forward_callback(self, msg):
        """
        Callback to decrypt incoming messages and forward them to the cmd_vel topic.
        """
        # Parse the encrypted message (assumes a specific format)
        encrypted_data, nonce = self.parse_encrypted_message(msg.data)

        decrypted_message = decrypt_and_verify(self.aesgcm, encrypted_data, nonce)
        if decrypted_message is None:
            self.get_logger().error("Failed to decrypt message or GMAC verification failed.")
            return
        # Log the decrypted message
        self.get_logger().info(f"Decrypted message: {decrypted_message}")

        # Parse the decrypted message into a Twist command
        twist_msg = self.parse_twist_command(decrypted_message)

        for publisher in self.publisher_list:
            try:
                publisher.publish(twist_msg)
                self.get_logger().info(f"Published to {publisher.topic_name} linear={twist_msg.linear.x}, angular={twist_msg.angular.z}")
            except Exception as e:
                self.get_logger().error(f'Error sending to {publisher.topic_name} {str(e)}')

    def parse_encrypted_message(self, data):
        """
        Parse the incoming encrypted message to extract the encrypted data and nonce.
        Assumes the message is formatted as 'nonce|encrypted_data' (base64-encoded).
        """
        try:
            nonce_b64, encrypted_data_b64 = data.split('|')
            nonce = base64.b64decode(nonce_b64)
            encrypted_data = base64.b64decode(encrypted_data_b64)
            return encrypted_data, nonce
        except Exception as e:
            raise ValueError(f"Invalid message format: {e}")

    def parse_twist_command(self, decrypted_message):
        """
        Parse the decrypted message into a Twist command.
        Assumes the message is formatted as 'linear_x,linear_y,linear_z,angular_x,angular_y,angular_z'.
        """
        try:
            values = [float(v) for v in decrypted_message.split(',')]
            twist_msg = Twist()
            twist_msg.linear.x = values[0]
            twist_msg.angular.z = values[1]
            print(twist_msg)
            return twist_msg
        except Exception as e:
            raise ValueError(f"Invalid Twist command format: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SecurityNode()

    try:
        # Wait for key exchange to complete
        print("Waiting for key exchange to complete...")
        while node.aes_key is None:
            rclpy.spin_once(node, timeout_sec=0.1)  # Spin the node to process callbacks
        print("Key exchange completed. AES key derived.")

        # Set up AES-GCM with the derived AES key
        node.aesgcm = AESGCM(node.aes_key)
        print("AES-GCM initialized with the derived key.")

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()