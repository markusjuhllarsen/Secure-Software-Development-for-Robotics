
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup

from controller.actions import RobotActionManager

import base64

from security_node import SecurityNode

class RobotSecurityNode(SecurityNode):
    def __init__(self, enable_security = True):
        super().__init__('security_node', enable_security)
        self.get_logger().info("Initializing Security Node...")

        self._init_publishers()
        self._init_subscribers()

        # Use reentrant callback group for actions
        self.callback_group = ReentrantCallbackGroup()

        # Initialize docking manager
        self.action_manager = RobotActionManager(self)

    def _init_publishers(self):
        self.publisher_list = []
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
            self.create_subscription(
                String, 
                '/encrypted_msg', 
                self.decrypt_and_forward_callback, 
                10
            )

    def decrypt_and_forward_callback(self, msg):
        """
        Callback to decrypt incoming messages and forward them to the cmd_vel topic.
        """
        # Parse the encrypted message (assumes a specific format)
        encrypted_data, nonce = self.parse_encrypted_message(msg.data)

        decrypted_message = self.decrypt_and_verify(encrypted_data, nonce)
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
            return twist_msg
        except Exception as e:
            raise ValueError(f"Invalid Twist command format: {e}")
