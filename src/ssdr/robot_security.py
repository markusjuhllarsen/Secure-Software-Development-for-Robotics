
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup

import base64

from security_node import SecurityNode
from actions import RobotActionManager

class RobotSecurityNode(SecurityNode):
    def __init__(self, enable_security = True):
        super().__init__('robot_security_node', enable_security)
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
            self.topic_forward_callback, 
            10
        )

    def topic_forward_callback(
            self, 
            encrypted_msg: String
            ) -> None:
        """
        Callback to decrypt incoming messages and forward them to the cmd_vel topics.
        args:
            encrypted_msg: The incoming encrypted message containing the nonce and encrypted data.
        """
        decrypted_message = self.decrypt_and_verify(encrypted_msg.data)
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

    def parse_twist_command(
            self, 
            decrypted_message: str
            ) -> Twist:
        """
        Parse the decrypted message into a Twist command.
        Assumes the message is formatted as 'linear_x,angular_z'.
        args:
            decrypted_message: The decrypted message containing the Twist command.
        """
        try:
            values = [float(v) for v in decrypted_message.split(',')]
            twist_msg = Twist()
            twist_msg.linear.x = values[0]
            twist_msg.angular.z = values[1]
            return twist_msg
        except Exception as e:
            raise ValueError(f"Invalid Twist command format: {e}")
