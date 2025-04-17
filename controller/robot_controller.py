#!/usr/bin/env python3

import subprocess
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

from controller.security import SecurityManager
from controller.actions import DockingManager
from utils.config import CMD_VEL_TOPICS

class SecureTurtlebot4Controller(Node):
    """Main controller class for the Turtlebot4 with security features"""
    
    def __init__(self):
        super().__init__('secure_turtlebot4_controller')
        
        # Print node and topic info at startup
        self.get_logger().info("Starting Secure Turtlebot4 Controller - if controls don't work, check topic names")
        
        # Use reentrant callback group for actions
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize security manager
        self.security = SecurityManager()
        
        # Initialize publishers
        self._init_publishers()
        
        # Initialize docking manager
        self.docking = DockingManager(self)
        
        # Topic discovery timer
        self.topic_check_done = False
        self.timer = self.create_timer(5.0, self.print_topics)
        
        # Reference to GUI (will be set in main.py)
        self.gui = None
        
        self.get_logger().info('Secure Turtlebot4 Controller initialized')
    
    def _init_publishers(self):
        """Initialize all publishers"""
        # Command publisher for movement - using multiple topics for reliability
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Root cmd_vel topic
            10
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
        
        self.status_subscriber = self.create_subscription(
            String,
            '/turtlebot4/status_msg', 
            self.status_callback,
            10
        )
    
    def status_callback(self, msg):
        """
        Callback for status messages from the robot
        """
        # Log status messages from the robot
        self.get_logger().info(f'Robot Status: {msg.data}')
        
        # Update GUI with the status message
        if hasattr(self, 'gui') and self.gui:
            self.gui.update_status(f"Robot: {msg.data}")
    
    def print_topics(self):
        """Print available topics to help with debugging"""
        try:
            # Only run once
            if hasattr(self, 'topic_check_done') and self.topic_check_done:
                return
                
            self.get_logger().info("Checking available topics...")
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
            topics = result.stdout.strip().split('\n')
            
            # Check for cmd_vel related topics specifically
            cmd_vel_topics = [t for t in topics if 'cmd_vel' in t]
            self.get_logger().info(f"Available cmd_vel topics: {cmd_vel_topics}")
            self.get_logger().info("Control ready. Use the GUI buttons to move the robot.")
            
            # Mark as done so it doesn't run again
            if hasattr(self, 'topic_check_done'):
                self.topic_check_done = True
                
        except Exception as e:
            self.get_logger().error(f"Error checking topics: {e}")
    
    def move_robot(self, linear_x, angular_z):
        """
        Send movement command with specified linear and angular velocities
        Publishing to all cmd_vel topics to ensure it works
        """
        # Security validation
        if not self.security.validate_command(f"move:{linear_x}:{angular_z}"):
            return False
            
        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        
        # Try publishing to ALL cmd_vel topics
        for topic in CMD_VEL_TOPICS:
            try:
                # Create a publisher for this topic
                temp_pub = self.create_publisher(Twist, topic, 10)
                temp_pub.publish(twist_msg)
                self.get_logger().info(f"Published to {topic}: linear={linear_x}, angular={angular_z}")
            except Exception as e:
                self.get_logger().error(f'Error sending to {topic}: {str(e)}')
        
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
        else:
            status = f"Moving: linear={linear_x}, angular={angular_z}"
            
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
    
    def dock_robot(self):
        """Proxy method to the docking manager's dock method"""
        return self.docking.dock_robot()
    
    def undock_robot(self):
        """Proxy method to the docking manager's undock method"""
        return self.docking.undock_robot()