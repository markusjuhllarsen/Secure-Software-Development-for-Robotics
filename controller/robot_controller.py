#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

from controller.security import SecurityManager
from controller.actions import DockingManager

class SecureTurtlebot4Controller(Node):
    """Main controller class for the Turtlebot4 with security features"""
    
    def __init__(self):
        super().__init__('secure_turtlebot4_controller')
        
        # Print node and topic info at startup
        self.get_logger().info("Starting Secure Turtlebot4 Controller - if controls don't work, check topic names")
        
        # Reference to GUI (will be set in main.py)
        self.gui = None

        # Use reentrant callback group for actions
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize security manager
        self.security = SecurityManager(self.get_logger())
        
        # Initialize publishers
        self._init_publishers()
        
        # Initialize docking manager
        self.docking = DockingManager(self)
        
        self.get_logger().info('Secure Turtlebot4 Controller initialized')
    
    def _init_publishers(self):
        """Initialize all publishers"""
        # Command publisher for movement - using multiple topics for reliability
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',  # Root cmd_vel_unstamped topic
            10 # Queue size
        )
        
        # Secondary publisher for the differential drive controller
        self.diff_drive_publisher = self.create_publisher(
            Twist,
            '/diffdrive_controller/cmd_vel',
            10
        )
        
        # Status publisher/subscriber for monitoring
        self.status_publisher = self.create_publisher(
            String,
            '/turtlebot4/status_msg',  # Status topic
            10
        )
        
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
        if not self.security.rate_limit_and_sanitize_command(linear_x, angular_z):
            return False
            
        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)

        try:
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info(f"Published to cmd_vel_unstamped: linear={linear_x}, angular={angular_z}")
        except Exception as e:
            self.get_logger().error(f'Error sending to cmd_vel_unstamped: {str(e)}')

        try:
            self.diff_drive_publisher.publish(twist_msg)
            self.get_logger().info(f"Published to diffdrive_controller/cmd_vel topic: linear={linear_x}, angular={angular_z}")
        except Exception as e:
            self.get_logger().error(f'Error sending to diffdrive_controller/cmd_vel: {str(e)}')
        
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
    
    def publish_status(self, status_text):
        """
        Publish status message and update the GUI
        """
        # Log the status
        self.get_logger().info(f"Status: {status_text}")
        
        # Try to publish to the topic if the publisher exists
        if hasattr(self, 'status_publisher'):
            try:
                msg = String()
                msg.data = status_text
                self.status_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing status: {str(e)}")
        
        # Update the GUI status window
        if hasattr(self, 'gui') and self.gui:
            self.gui.update_status(status_text)