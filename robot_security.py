from utils.setup import setup_ros2_environment

setup_ros2_environment()

import rclpy
from rclpy.node import Node
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import secrets
import base64

from geometry_msgs.msg import Twist  # For publishing to cmd_vel
from std_msgs.msg import String  # For receiving encrypted messages

from utils.security import decrypt_and_verify

class SecurityNode(Node):
    def __init__(self):
        super().__init__('security_node')
        self.get_logger().info("Initializing Security Node...")

        # Security setup
        self.secret_key = secrets.token_bytes(32)  # Replace with a shared key
        self.secret_key = b'#\x9a$\x80\xb0Z\x82\x17\xe0/\x16\xa5V\xfa\xd7\xa38\x1b\xab\xe4I\xb4\x1bnPW\xb7A\xc9DQ\xa6'
        self.aesgcm = AESGCM(self.secret_key)

        self.publisher_list = []
        self._init_publishers()

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
    
    def _init_subscribers(self):
        # Subscriptions and publishers
        self.encrypted_subscriber = self.create_subscription(
            String,
            'encrypted_topic',
            self.decrypt_and_forward_callback,
            10
        )

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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()