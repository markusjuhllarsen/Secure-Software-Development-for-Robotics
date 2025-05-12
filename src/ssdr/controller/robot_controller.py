#!/usr/bin/env python3

from datetime import datetime
import base64

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from controller.sanitizer_manager import Sanitizer


from actions import RobotActionManager
from security_node import SecurityNode

class SecureTurtlebot4Controller(SecurityNode):
    """Main controller class for the Turtlebot4 with security features"""
    
    def __init__(self, enable_security = False):
        super().__init__('secure_turtlebot4_controller', enable_security, is_controller = True)
        
        # Reference to GUI (will be set in main.py)
        self.gui = None

        # Use reentrant callback group for actions
        self.callback_group = ReentrantCallbackGroup()

        # Initialize non-encrypted publishers
        self._init_publishers()

        # Initialize docking manager
        self.action_manager = RobotActionManager(self)
        
        self.get_logger().info('Secure Turtlebot4 Controller initialized.')
    
    def _init_publishers(self):
        """Initialize all publishers"""
        self.publisher_list = []

        if not self.enable_security:

            # Command publisher for movement - using multiple topics for reliability
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
            
            # Status publisher/subscriber for monitoring
            self.status_publisher = self.create_publisher(
                String,
                '/turtlebot4/status_msg',  # Status topic
                10
            )
        
        else:
            # If security is enabled, use the security node's publisher
            self.publisher_list.append(self.create_publisher(
                String,
                '/encrypted_msg',  # Encrypted message topic
                10
            ))
    
        #self.status_subscriber = self.create_subscription(
        #    String,
        #    '/turtlebot4/status_msg', 
        #    self.status_callback,
        #    10
        #)
    
    def status_callback(self, msg):
        """
        Callback for status messages from the robot
        """
        # Log status messages from the robot
        self.get_logger().info(msg.data)
        
        # Update GUI with the status message
        if hasattr(self, 'gui') and self.gui:
            self.gui.update_status("msg.data")
    
    def move_robot(self, linear_x, angular_z):
        """
        Send movement command with specified linear and angular velocities
        Publishing to all cmd_vel topics to ensure it works
        """
        # Security validation
        #if not self.security.rate_limit_and_sanitize_command(linear_x, angular_z):
        #    return False
        
        # Sanitize inputs
        linear_x = Sanitizer.sanitize_velocity(linear_x, min_value=-1.0, max_value=1.0, name="linear_x")
        angular_z = Sanitizer.sanitize_velocity(angular_z, min_value=-1.0, max_value=1.0, name="angular_z")


        if self.enable_security:
            timestamp = datetime.now().isoformat()
            message = f"{linear_x},{angular_z}"
            nonce, encrypted = self.encrypt_and_gmac(message, timestamp)

            nonce_b64 = base64.b64encode(nonce).decode()
            encrypted_data_b64 = base64.b64encode(encrypted).decode()
            encrypted_msg = f"{nonce_b64}|{encrypted_data_b64}"
            message = String()
            message.data = encrypted_msg
        else:
            # Create Twist message
            message = Twist()
            message.linear.x = linear_x
            message.angular.z = angular_z
        for publisher in self.publisher_list:
            try:
                publisher.publish(message)
                self.get_logger().info(f"Published to {publisher.topic_name} linear={linear_x}, angular={angular_z}")
            except Exception as e:
                self.get_logger().error(f'Error sending to {publisher.topic_name} {str(e)}')
        
        # Log the command and update status
        self.get_logger().info(f"Movement command sent to all topics: linear={linear_x}, angular={angular_z}")
        
        # Create a human-readable status message
        if linear_x > 0 and angular_z == 0:
            status = "Moving forward"
        elif linear_x < 0 and angular_z == 0:
            status = "Moving backward"
        elif linear_x == 0 and angular_z > 0:
            status = "Turning left"
        elif linear_x == 0 and angular_z < 0:
            status = "Turning right"
        elif linear_x == 0 and angular_z == 0:
            status = "Stopped"
        #else:
        #    status = f"Moving: linear={linear_x}, angular={angular_z}" #!!!!
            
        self.publish_status(status)
        return True
