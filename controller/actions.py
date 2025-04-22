#!/usr/bin/env python3

from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock, NavigateToPosition, RotateAngle
from geometry_msgs.msg import PoseStamped
import math


class RobotActionManager:
    """
    Generic action manager for Turtlebot4 that handles various robot actions
    """

    def __init__(self, node):
        """
        Initialize the action manager

        Args:
            node: The ROS2 node that will own the action clients
            enable_security: Whether to enable security checks
        """
        self.node = node
        self.action_clients = {}

        # Initialize action clients
        self._init_action_clients()

    def _init_action_clients(self):
        """Initialize all action clients needed for the robot"""
        # Docking actions
        self.action_clients["dock"] = ActionClient(
            self.node,
            Dock,
            '/dock',
            callback_group=self.node.callback_group
        )

        self.action_clients["undock"] = ActionClient(
            self.node,
            Undock,
            '/undock',
            callback_group=self.node.callback_group
        )

        # Navigation actions
        self.action_clients["navigate"] = ActionClient(
            self.node,
            NavigateToPosition,
            '/navigate_to_position',
            callback_group=self.node.callback_group
        )

        # Rotation action
        self.action_clients["rotate"] = ActionClient(
            self.node,
            RotateAngle,
            '/rotate_angle',
            callback_group=self.node.callback_group
        )


    def _validate_command(self, command):
        """
        Validate command if security is enabled

        Args:
            command: The command to validate

        Returns:
            bool: True if command is valid or security is disabled
        """
        #if not self.enable_security:
        #    return True

        #if hasattr(self.node, 'security'):
        #    return self.node.security.rate_limit_and_sanitize_command(command)

        return True

    def _send_action(self, action_name, goal_msg, response_callback):
        """
        Generic method to send an action to the robot

        Args:
            action_name: Name of the action to send
            goal_msg: The goal message for the action
            response_callback: Callback for handling the response

        Returns:
            bool: True if the action was sent successfully
        """
        if not self._validate_command(action_name):
            return False

        self.node.get_logger().info(f"Sending {action_name} action...")

        # Check if action client exists
        if action_name not in self.action_clients:
            self.node.get_logger().error(f"{action_name} action client not initialized!")
            self.node.publish_status(f"{action_name} action not available!")
            return False

        # Check if action server is available
        client = self.action_clients[action_name]
        if not client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(f"{action_name} action server not available!")
            self.node.publish_status(f"{action_name} action server not available!")
            return False

        # Send goal
        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(response_callback)
        self.node.publish_status(f"{action_name} action sent")
        return True

    # Dock robot action
    def dock_robot(self):
        """Send dock action to robot"""
        goal_msg = Dock.Goal()
        return self._send_action("dock", goal_msg, self._dock_response_callback)

    def _dock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Dock goal rejected!")
            self.node.publish_status("Dock goal rejected!")
            return

        self.node.get_logger().info("Dock goal accepted, waiting for result...")
        self.node.publish_status("Dock goal accepted, waiting for result...")

        # Request result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._dock_result_callback)

    def _dock_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f"Dock completed with result: {result}")
        self.node.publish_status("Dock completed")

    # Undock robot action
    def undock_robot(self):
        """Send undock action to robot"""
        goal_msg = Undock.Goal()
        return self._send_action("undock", goal_msg, self._undock_response_callback)

    def _undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Undock goal rejected!")
            self.node.publish_status("Undock goal rejected!")
            return

        self.node.get_logger().info("Undock goal accepted, waiting for result...")
        self.node.publish_status("Undock goal accepted, waiting for result...")

        # Request result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._undock_result_callback)

    def _undock_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f"Undock completed with result: {result}")
        self.node.publish_status("Undock completed")

    def navigate_to_pose(self, x, y, theta=0.0):
        """
        Use the nav2 action to navigate to a pose
        This uses the ROS2 Navigation2 stack if available

        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians
        """
        # Try to import the nav2 messages
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError:
            self.node.get_logger().error("Nav2 messages not available. Is navigation2 installed?")
            self.node.publish_status("Navigation2 not available")
            return False

        # Initialize the action client if not already done
        if "nav2" not in self.action_clients:
            self.action_clients["nav2"] = ActionClient(
                self.node,
                NavigateToPose,
                'navigate_to_pose',
                callback_group=self.node.callback_group
            )

        # Create the goal
        goal_msg = NavigateToPose.Goal()

        # Set the goal position
        pose = PoseStamped()
        # Don't set the timestamp at all - let Nav2 handle it
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)

        goal_msg.pose = pose

        # Send the action
        return self._send_action("nav2", goal_msg, self._nav2_response_callback)

    def _navigate_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Navigation goal rejected!")
            self.node.publish_status("Navigation goal rejected!")
            return

        self.node.get_logger().info("Navigation goal accepted, moving to position...")
        self.node.publish_status("Moving to position...")

        # Request result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._navigate_result_callback)

    def _navigate_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f"Navigation completed: {result}")
        self.node.publish_status("Navigation completed")

    def rotate_angle(self, angle_rad=1.57, max_rotation_speed=0.5):
        """
        Rotate the robot by a specific angle using cmd_vel_unstamped

        Args:
            angle_rad: Angle to rotate in radians (positive is counterclockwise, negative is clockwise)
            max_rotation_speed: Maximum rotation speed in rad/s (should be positive)

        Returns:
            bool: True if the command was sent successfully
        """
        from geometry_msgs.msg import Twist
        import time
        import threading
        import math

        # Create a publisher for unstamped velocity commands if it doesn't exist
        if not hasattr(self, 'unstamped_vel_publisher'):
            self.unstamped_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel_unstamped',  # Unstamped velocity commands topic
                10
            )

        # Ensure positive rotation speed and determine direction
        direction = 1 if angle_rad >= 0 else -1
        rotation_speed = abs(max_rotation_speed) * direction
        angle = abs(angle_rad)

        # Create rotation command
        twist = Twist()
        twist.angular.z = float(rotation_speed)

        # Add a correction factor for real-world rotation
        # Robots often need more time to complete rotations accurately
        correction_factor = 1.2

        # Calculate duration
        duration_sec = (angle / abs(rotation_speed)) * correction_factor

        # Log the command
        self.node.get_logger().info(
            f"Rotating {'counterclockwise' if direction > 0 else 'clockwise'} by {angle:.2f} radians ({math.degrees(angle):.1f}°)")
        self.node.get_logger().info(
            f"Using correction factor {correction_factor:.2f}, calculated duration: {duration_sec:.2f}s")
        self.node.publish_status(
            f"Rotating {'counterclockwise' if direction > 0 else 'clockwise'} by {math.degrees(angle):.1f}°...")

        # Flag to control the execution loop
        if not hasattr(self, 'movement_active'):
            self.movement_active = False

        # Stop any existing movement first
        if self.movement_active:
            self.movement_active = False
            time.sleep(0.5)  # Give time for existing movement to stop

        # Define the rotation control in a separate thread
        def execute_rotation():
            # Set the active flag
            self.movement_active = True

            # Calculate the end time
            start_time = time.time()
            end_time = start_time + duration_sec

            # Acceleration phase - 0.5 second
            accel_start = time.time()
            accel_duration = 0.5  # 0.5 second acceleration

            # Track current velocity for smooth acceleration/deceleration
            current_velocity = 0.0
            target_velocity = rotation_speed

            # Main movement loop with high frequency updates (50Hz)
            refresh_rate = 0.02  # 20ms refresh (50Hz)

            try:
                while self.movement_active and time.time() < end_time:
                    current_time = time.time()

                    # Calculate current target velocity based on phase
                    if current_time < accel_start + accel_duration:
                        # Acceleration phase
                        progress = (current_time - accel_start) / accel_duration
                        current_velocity = target_velocity * min(1.0, progress)
                    elif current_time > end_time - 0.5:
                        # Deceleration phase (last 0.5 second)
                        progress = (end_time - current_time) / 0.5
                        current_velocity = target_velocity * max(0.0, progress)
                    else:
                        # Constant velocity phase
                        current_velocity = target_velocity

                    # Create and send velocity command
                    cmd = Twist()
                    cmd.angular.z = float(current_velocity)
                    self.unstamped_vel_publisher.publish(cmd)

                    # Short sleep for smooth control
                    time.sleep(refresh_rate)

                # Final stop command
                stop_cmd = Twist()
                for _ in range(5):
                    self.unstamped_vel_publisher.publish(stop_cmd)
                    time.sleep(0.02)

            except Exception as e:
                self.node.get_logger().error(f"Error in rotation thread: {str(e)}")
            finally:
                # Always send stop command when done or if exception occurs
                stop_cmd = Twist()
                for _ in range(5):
                    self.unstamped_vel_publisher.publish(stop_cmd)
                    time.sleep(0.02)

                self.movement_active = False
                self.node.publish_status("Rotation completed")

        # Start the rotation in a background thread
        rotation_thread = threading.Thread(target=execute_rotation)
        rotation_thread.daemon = True
        rotation_thread.start()

        return True

    def drive_distance_unstamped(self, distance=1.0, velocity=0.2):
        """
        Move forward by a specified distance using cmd_vel_unstamped topic
        With smooth continuous motion and adjusted correction factor

        Args:
            distance: Distance to travel in meters (positive for forward, negative for backward)
            velocity: Linear velocity in m/s (should be positive)
        """
        from geometry_msgs.msg import Twist
        import time
        import threading

        # Create a publisher for unstamped velocity commands if it doesn't exist
        if not hasattr(self, 'unstamped_vel_publisher'):
            self.unstamped_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel_unstamped',  # Unstamped velocity commands topic
                10
            )

        # Use positive velocity and adjust sign based on distance
        direction = 1 if distance >= 0 else -1
        velocity = abs(velocity) * direction
        distance = abs(distance)

        # Create velocity command
        twist = Twist()
        twist.linear.x = float(velocity)

        # Reduced correction factor - about 25% less than previous implementation
        if distance <= 1.0:
            correction_factor = 1.1  # Reduced from 1.2
        elif distance <= 2.0:
            correction_factor = 1.2  # Reduced from 1.4
        else:
            # Reduced multiplier from 0.16 to 0.12 (25% reduction)
            correction_factor = 1.1 + (distance * 0.08)

        # Calculate duration with the correction factor
        duration_sec = (distance / abs(velocity)) * correction_factor

        # Log the command with correction factor info
        self.node.get_logger().info(
            f"Moving {'forward' if direction > 0 else 'backward'} {distance}m at {abs(velocity)}m/s")
        self.node.get_logger().info(
            f"Using correction factor {correction_factor:.2f}, calculated duration: {duration_sec:.2f}s")
        self.node.publish_status(f"Moving {'forward' if direction > 0 else 'backward'} {distance}m...")

        # Flag to control the execution loop
        if not hasattr(self, 'movement_active'):
            self.movement_active = False

        # Stop any existing movement first
        if self.movement_active:
            self.movement_active = False
            time.sleep(0.5)  # Give time for existing movement to stop

        # Define the motion control in a separate thread
        def execute_movement():
            # Set the active flag
            self.movement_active = True

            # Calculate the end time
            start_time = time.time()
            end_time = start_time + duration_sec

            # Acceleration phase - 1 second
            accel_start = time.time()
            accel_duration = 1.0  # 1 second acceleration

            # Track current velocity for smooth acceleration/deceleration
            current_velocity = 0.0
            target_velocity = velocity

            # Main movement loop with high frequency updates (50Hz)
            refresh_rate = 0.02  # 20ms refresh (50Hz)

            try:
                while self.movement_active and time.time() < end_time:
                    current_time = time.time()

                    # Calculate current target velocity based on phase
                    if current_time < accel_start + accel_duration:
                        # Acceleration phase
                        progress = (current_time - accel_start) / accel_duration
                        current_velocity = target_velocity * min(1.0, progress)
                    elif current_time > end_time - 1.0:
                        # Deceleration phase (last 1 second)
                        progress = (end_time - current_time) / 1.0
                        current_velocity = target_velocity * max(0.0, progress)
                    else:
                        # Constant velocity phase
                        current_velocity = target_velocity

                    # Create and send velocity command
                    cmd = Twist()
                    cmd.linear.x = float(current_velocity)
                    self.unstamped_vel_publisher.publish(cmd)

                    # Short sleep for smooth control
                    time.sleep(refresh_rate)

                # Final stop command
                stop_cmd = Twist()
                for _ in range(5):
                    self.unstamped_vel_publisher.publish(stop_cmd)
                    time.sleep(0.02)

            except Exception as e:
                self.node.get_logger().error(f"Error in movement thread: {str(e)}")
            finally:
                # Always send stop command when done or if exception occurs
                stop_cmd = Twist()
                for _ in range(5):
                    self.unstamped_vel_publisher.publish(stop_cmd)
                    time.sleep(0.02)

                self.movement_active = False
                self.node.publish_status("Movement completed")

        # Start the movement in a background thread
        movement_thread = threading.Thread(target=execute_movement)
        movement_thread.daemon = True
        movement_thread.start()

        return True