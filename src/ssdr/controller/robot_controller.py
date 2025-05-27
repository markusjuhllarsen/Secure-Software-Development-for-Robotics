#!/usr/bin/env python3

from datetime import datetime
import tkinter as tk

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

from actions import RobotActionManager
from security_node import SecurityNode
from controller.sanitizer_manager import Sanitizer

class SecureTurtlebot4Controller(SecurityNode):
    """Main controller class for the Turtlebot4 with security features"""
    
    def __init__(
            self, 
            enable_security: bool = False
            ) -> None:
        super().__init__('secure_turtlebot4_controller', enable_security, is_controller = True)
        
        # Reference to GUI (will be set in main.py)
        self.gui = None

        self.movement_timer = None
        self.current_command = None  # Store the current movement command

        # Use reentrant callback group for actions
        self.callback_group = ReentrantCallbackGroup()

        # Initialize non-encrypted publishers
        self._init_publishers()

        # Initialize action manager
        self.action_manager = RobotActionManager(self)
        
    def _init_publishers(
            self
            ) -> None:
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
        else:
            # If security is enabled, use the security node's publisher
            self.publisher_list.append(self.create_publisher(
                String,
                '/encrypted_msg',  # Encrypted message topic
                10
            ))

    def start_repeating_command(
            self, 
            linear_x: float, 
            angular_z: float
            ) -> None:
        """
        Start repeating the movement command at a fixed interval.
        All other buttons (except stop) are disabled while this is active.
        args:
            linear_x: Linear velocity in the x direction.
            angular_z: Angular velocity around the z axis.
        """
        if not Sanitizer.check_rate_limit('movement'):  # Check rate limit before proceeding
            self.publish_status("Rate limit exceeded. Please wait before sending another command.")
            return

        self.current_command = (linear_x, angular_z)
        if self.movement_timer is None:
            self.movement_timer = self.create_timer(0.7, self._repeat_command)  # Publish every 0.1 seconds
        if linear_x > 0 and angular_z == 0:
            status = "Moving forward"
        elif linear_x < 0 and angular_z == 0:
            status = "Moving backward"
        elif linear_x == 0 and angular_z > 0:
            status = "Turning left"
        elif linear_x == 0 and angular_z < 0:
            status = "Turning right"
        
        self.publish_status(status)

        for button in self.gui.action_buttons.values():
            button.config(state=tk.DISABLED)
        
    def stop_repeating_command(
            self
            ) -> None:
        """
        Stop repeating the movement command.
        Enable all buttons again.
        """
        if self.movement_timer is not None:
            self.movement_timer.cancel()
            self.movement_timer = None
            self.publish_status("Stopped")
            self.move_robot(0, 0) # Stop the robot

            for button in self.gui.action_buttons.values():
                button.config(state=tk.NORMAL)

    def _repeat_command(
            self
            ) -> None:
        """
        Publish the current movement command repeatedly.
        """
        if self.current_command is not None:
            linear_x, angular_z = self.current_command
            self.move_robot(linear_x, angular_z)

    def move_robot(
            self, 
            linear_x: float, 
            angular_z: float
            ) -> None:
        """
        Send movement command with specified linear and angular velocities
        Publishing to all cmd_vel topics to ensure it works
        args:
            linear_x: Linear velocity in the x direction.
            angular_z: Angular velocity around the z axis.
        """
        # If sanitization returns None or False, do not proceed
        if linear_x is False and angular_z is False:
            self.publish_status(linear_x)
            self.publish_status(angular_z)
            self.publish_status("Rate limit exceeded or invalid input.")
            return
        
        # Ensure values are floats
        linear_x = float(linear_x)
        angular_z = float(angular_z)

        if self.enable_security:
            timestamp = datetime.now().isoformat()
            message = f"{linear_x},{angular_z}"
            encrypted_msg= self.encrypt_and_gmac(message, timestamp)
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