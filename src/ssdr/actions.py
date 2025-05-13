#!/usr/bin/env python3


from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from ssdr_interfaces.action import EncryptedAction
from irobot_create_msgs.action import Dock, Undock, DriveArc, DriveDistance, NavigateToPosition, RotateAngle, WallFollow
from geometry_msgs.msg import PoseStamped
import math
import time

class RobotActionManager:
    """
    Generic action manager for Turtlebot4 that handles various robot actions
    """
    def __init__(self, node, update_button_callback = None):
        """
        Initialize the action manager

        Args:
            node: The ROS2 node that will own the action clients
        """
        self.node = node
        self.actions = ["Dock", "Undock", "DriveArc", "DriveDistance", "NavigateToPosition", "RotateAngle", "WallFollow"]
        self.active_goals = {}
        self.update_button_callback = update_button_callback

        # Initialize action clients
        if self.node.is_controller:
            if self.node.enable_security:
                self._init_encrypted_action_clients()
            else:
                self._init_unencrypted_action_clients()
        else:
            self._init_forwarding_action_servers()
            self._init_unencrypted_action_clients()

    def _init_unencrypted_action_clients(self):
        """Initialize all unencrypted action clients needed for the robot"""
        self.action_clients = {}
        for action_name in self.actions:
            self.action_clients[action_name] = ActionClient(
                self.node,
                globals()[action_name],
                f'/{action_name.lower()}',
                callback_group=self.node.callback_group
            )

    def _init_encrypted_action_clients(self):
        """Initialize all encrypted action clients""" 
        self.encrypted_client = ActionClient(
            self.node,
            EncryptedAction,
            f'/encrypted_forward',
            callback_group=self.node.callback_group
        )
    
    def _init_forwarding_action_servers(self):
        """Initialize all encrypted action servers needed for decrypting and forwarding actions"""         
        # Goal callback must return immediately, so we accept all goals
        # and defer further goal acceptance handling to execute callback
        self.forward_server = ActionServer(
            self.node,
            EncryptedAction,
            '/encrypted_forward',
            execute_callback=self._forward_callback,
            cancel_callback=lambda goal_handle: self._cancel_callback(goal_handle),
        )

    # TODO: Cancel encryption
    # TODO: Remove extra encrypted action clients
    # TODO: Add more actions

    async def _forward_callback(self, goal_handle):
        """
        When recieving encrypted action, decrypt it and forward it to the appropriate unencrypted client.
        Args:
            goal_handle: The goal handle for the action server.
        Returns:
            EncryptedAction.Result: The result of the action.
        """
        action_name = goal_handle._goal_request.action_name
        action_name = self.node.decrypt_and_verify(action_name)
        goal_msg = goal_handle._goal_request.encrypted_goal
        goal_msg = self.node.decrypt_and_verify(goal_msg)
        goal_msg = globals()[action_name].Goal()   

        self.node.get_logger().info(f"Forwarding {action_name} action.")  

        # Send the goal asynchronously
        client = self.action_clients[action_name]
        client.wait_for_server()
        send_goal_future = client.send_goal_async(goal_msg)
        # Wait for the goal to be sent
        goal_handle_client = await send_goal_future
        if not goal_handle_client.accepted:
            self.node.get_logger().error(f"{action_name} action rejected.")
            goal_handle.abort()
            return globals()[action_name].Result()

        self.active_goals[id(goal_handle)] = goal_handle_client

        self.node.get_logger().info(f"{action_name} action goal accepted by the client.")

        # Wait for the result asynchronously
        get_result_future = goal_handle_client.get_result_async()
        result = await get_result_future

        del self.active_goals[id(goal_handle)]

        if result.status == 4:  # SUCCEEDED
            self.node.publish_status(f"{action_name} action succeeded.")
            goal_handle.succeed()  # Mark the server goal as succeeded
        elif result.status == 5:  # CANCELED
            self.node.publish_status(f"{action_name} action canceled.")
            goal_handle.canceled()  # Mark the server goal as canceled
        else:
            self.node.publish_status(f"{action_name} action aborted.")
            goal_handle.abort()  # Mark the server goal as aborted
        timestamp = time.time
        encrypted_result = self.node.encrypt_and_gmac(str(result.result), timestamp)
        return_result = EncryptedAction.Result()
        return_result.encrypted_result = encrypted_result
        return return_result
    
    def _cancel_callback(self, goal_handle):
        """
        Handle cancellation requests for actions.
        Args:
            goal_handle: The goal handle for the action server.
        Returns:
            CancelResponse: ACCEPT or REJECT.
        """
        action_name = goal_handle._goal_request.action_name
        action_name = self.node.decrypt_and_verify(action_name)
        cancel_client_goal_handle = None
        for server_goal_handle_id, client_goal_handle in self.active_goals.items():
            if server_goal_handle_id == id(goal_handle):
                cancel_client_goal_handle = client_goal_handle
                break
        
        if not cancel_client_goal_handle:
            self.node.get_logger().error("No matching client goal handle found for the goal handle.")
            return CancelResponse.REJECT
        
        self.node.publish_status(f"Cancelling {action_name} action.")
        cancel_future = cancel_client_goal_handle.cancel_goal_async()
        #cancel_future.add_done_callback(lambda future: self._cancel_done_callback(action_name, future))
        return CancelResponse.ACCEPT

    def _send_goal(self, action_name, goal_msg):
        """
        Generic method to send an action
        Args:
            action_name: Name of the action to send
            goal_msg: The goal message for the action
        """
        self.node.publish_status(f"Sending {action_name} action.")        
        if self.node.enable_security:
            client = self.encrypted_client
        else:     
            client = self.action_clients[action_name]
        client.wait_for_server()

        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(
            lambda future: self._response_callback(action_name, future)
        )

    def _response_callback(self, action_name, future):
        """
        Callback for when the send goal response is received
        Args:
            action_name: Name of the action
            future: The future object containing the result
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.publish_status(f'{action_name} action rejected.')
            return
         
        self.node.publish_status(f'{action_name} action accepted.')
        self.active_goals[action_name] = goal_handle

        if self.update_button_callback:
            # Goal accepted, can now cancel
            self.update_button_callback(action_name, "Cancel")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future: self._result_callback(action_name, future)
        )

    def _feedback_callback(self, feedback_msg):
        # Not implemented yet
        feedback = feedback_msg.feedback

    def _result_callback(self, action_name, future):
        """
        Callback for when the action result is received
        Args:
            action_name: Name of the action
            future: The future object containing the result
        """
        goal_handle = future.result()
        if goal_handle.status == 6:
            self.node.publish_status(f"{action_name} action rejected or aborted.")
        elif goal_handle.status == 5:
            self.node.publish_status(f"{action_name} action canceled.")     
        else:
            self.node.publish_status(f"{action_name} action completed.")
        
        del self.active_goals[action_name]
        
        if self.update_button_callback:
            # Goal finished, can no longer cancel
            self.update_button_callback(action_name, "Action")
        
        return goal_handle.result
    
    def cancel_action(self, action_name):
        """
        Cancel an active goal for the specified action.
        Args:
            action_name: Name of the action to cancel
        """
        if action_name not in self.active_goals:
            self.node.get_logger().error(f"{action_name} goal not active!")
            return

        goal_handle = self.active_goals[action_name]
        cancel_future = goal_handle.cancel_goal_async()
        #cancel_future.add_done_callback(lambda future: self._cancel_done_callback(action_name, future))   

    def _cancel_done_callback(self, action_name, future):
        """
        Callback for handling the result of a cancelled action
        Args:
            action_name: Name of the action being canceled.
            future: The future object containing cancel response.
        """
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info(f"{action_name} action successfully canceled.")
        else:
            self.node.get_logger().info(f"No active goals to cancel for {action_name}.")

    def dock_robot(self):
        """Send dock action to robot."""
        if self.node.enable_security:
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("Dock", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac('', timestamp)
        else:
            goal_msg = Dock.Goal()
        self._send_goal("Dock", goal_msg)
    
    def undock_robot(self):
        """Send undock action to robot."""
        if self.node.enable_security:
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("Undock", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac('', timestamp)
        else:
            goal_msg = Undock.Goal()
        self._send_goal("Undock", goal_msg)

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
        return self._send_goal("nav2", goal_msg, self._nav2_response_callback)

    def _navigate_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Navigation goal rejected!")
            self.node.publish_status("Navigation goal rejected!")
            return

        self.node.get_logger().info("Navigation goal accepted, moving to position.")
        self.node.publish_status("Moving to position.")

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
            f"Rotating {'counterclockwise' if direction > 0 else 'clockwise'} by {math.degrees(angle):.1f}°.")

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
        self.node.publish_status(f"Moving {'forward' if direction > 0 else 'backward'} {distance}m.")

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