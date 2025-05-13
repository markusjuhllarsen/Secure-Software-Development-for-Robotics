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
        self.actions = {
            "Dock": "/dock",
            "Undock": "/undock",
            "DriveArc": "/drive_arc",
            "DriveDistance": "/drive_distance",
            "NavigateToPosition": "/navigate_to_position",
            "RotateAngle": "/rotate_angle",
            "WallFollow": "/wall_follow",
        }
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
        for action_name , topic_name in self.actions.items():
            self.action_clients[action_name] = ActionClient(
                self.node,
                globals()[action_name],
                topic_name,
                callback_group=self.node.callback_group
            )

    def _init_encrypted_action_clients(self):
        """Initialize all encrypted action clients""" 
        self.encrypted_client = ActionClient(
            self.node,
            EncryptedAction,
            '/encrypted_forward',
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
        goal_str = goal_handle._goal_request.encrypted_goal
        goal_str = self.node.decrypt_and_verify(goal_str)
        goal_msg = self.parse_goal(action_name, goal_str)
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
    
    def parse_dock_goal(self, goal_str):
        """
        Parse the goal string for the Dock action.
        Args:
            goal_str (str): The goal string (empty for Dock).
        Returns:
            Dock.Goal: The constructed goal message.
        """
        return Dock.Goal()  # Dock action does not require additional parameters
    
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
    
    def parse_undock_goal(self, goal_str):
        """
        Parse the goal string for the Undock action.
        Args:
            goal_str (str): The goal string (empty for Undock).
        Returns:
            Undock.Goal: The constructed goal message.
        """
        return Undock.Goal()  # Undock action does not require additional parameters

    def drive_distance(self, distance=10.0, velocity=3.0):
        """
        Move the robot a specified distance using DriveDistance action

        Args:
            distance: Distance to travel in meters (positive for forward, negative for backward)
            velocity: Linear velocity in m/s (should be positive)
        """
        if self.node.enable_security:
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("DriveDistance", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac(f"{distance},{velocity}", timestamp)
        else:
            goal_msg = DriveDistance.Goal()
            goal_msg.distance = float(distance)
            goal_msg.max_translation_speed = float(velocity)

        self._send_goal("DriveDistance", goal_msg)
    
    def parse_drive_distance_goal(self, goal_str):
        """
        Parse the goal string for the DriveDistance action.
        Args:
            goal_str (str): The goal string in the format "distance,velocity".
        Returns:
            DriveDistance.Goal: The constructed goal message.
        """
        try:
            distance, velocity = map(float, goal_str.split(","))
            goal_msg = DriveDistance.Goal()
            goal_msg.distance = distance
            goal_msg.max_translation_speed = velocity
            return goal_msg
        except ValueError:
            raise ValueError(f"Invalid goal string for DriveDistance: {goal_str}")
    
    def drive_arc(self, translate_direction=-1, angle=2.57, radius=1.0, max_translation_speed=0.3):
        """
        Send a DriveArc action to the robot.

        Args:
            translate_direction (int): Direction of translation (1 for forward, -1 for backward).
            angle (float): Relative angle (radians) to rotate along the arc.
            radius (float): Radius of the arc (meters).
            max_translation_speed (float): Maximum translation speed (m/s).
        """
        if self.node.enable_security:
            # Encrypt the action name and goal
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("DriveArc", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac(
                f"{translate_direction},{angle},{radius},{max_translation_speed}", timestamp
            )
        else:
            # Create the DriveArc goal message
            goal_msg = DriveArc.Goal()
            goal_msg.translate_direction = int(translate_direction)
            goal_msg.angle = float(angle)
            goal_msg.radius = float(radius)
            goal_msg.max_translation_speed = float(max_translation_speed)

        # Send the goal
        self._send_goal("DriveArc", goal_msg)
    
    def parse_drive_arc_goal(self, goal_str):
        """
        Parse the goal string for the DriveArc action.

        Args:
            goal_str (str): The goal string in the format "translate_direction,angle,radius,max_translation_speed".

        Returns:
            DriveArc.Goal: The constructed goal message.
        """
        translate_direction, angle, radius, max_translation_speed = map(float, goal_str.split(","))
        goal_msg = DriveArc.Goal()
        goal_msg.translate_direction = int(translate_direction)
        goal_msg.angle = angle
        goal_msg.radius = radius
        goal_msg.max_translation_speed = max_translation_speed
        return goal_msg

    def navigate_to_position(self, x=1.0, y=2.0, theta=1.57, achieve_goal_heading=True, max_translation_speed=0.3, max_rotation_speed=1.9):
        """
        Send a NavigateToPosition action to the robot.

        Args:
            x (float): X-coordinate of the goal position in meters.
            y (float): Y-coordinate of the goal position in meters.
            theta (float): Orientation (yaw) of the goal position in radians.
            achieve_goal_heading (bool): Whether to achieve the goal heading for final orientation.
            max_translation_speed (float): Maximum translation speed (m/s).
            max_rotation_speed (float): Maximum rotation speed (rad/s).
        """
        if self.node.enable_security:
            # Encrypt the action name and goal
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("NavigateToPosition", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac(
                f"{x},{y},{theta},{achieve_goal_heading},{max_translation_speed},{max_rotation_speed}", timestamp
            )
        else:
            # Create the NavigateToPosition goal message
            goal_msg = NavigateToPosition.Goal()
            goal_msg.goal_pose = PoseStamped()
            goal_msg.goal_pose.header.frame_id = "map"
            goal_msg.goal_pose.pose.position.x = float(x)
            goal_msg.goal_pose.pose.position.y = float(y)
            goal_msg.goal_pose.pose.orientation.z = math.sin(theta / 2.0)
            goal_msg.goal_pose.pose.orientation.w = math.cos(theta / 2.0)
            goal_msg.achieve_goal_heading = bool(achieve_goal_heading)
            goal_msg.max_translation_speed = float(max_translation_speed)
            goal_msg.max_rotation_speed = float(max_rotation_speed)

        # Send the goal
        self._send_goal("NavigateToPosition", goal_msg)
    
    def parse_navigate_to_position_goal(self, goal_str):
        """
        Parse the goal string for the NavigateToPosition action.

        Args:
            goal_str (str): The goal string in the format "x,y,theta,achieve_goal_heading,max_translation_speed,max_rotation_speed".

        Returns:
            NavigateToPosition.Goal: The constructed goal message.
        """
        x, y, theta, achieve_goal_heading, max_translation_speed, max_rotation_speed = goal_str.split(",")
        goal_msg = NavigateToPosition.Goal()
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.header.frame_id = "map"
        goal_msg.goal_pose.pose.position.x = float(x)
        goal_msg.goal_pose.pose.position.y = float(y)
        goal_msg.goal_pose.pose.orientation.z = math.sin(float(theta) / 2.0)
        goal_msg.goal_pose.pose.orientation.w = math.cos(float(theta) / 2.0)
        goal_msg.achieve_goal_heading = bool(int(achieve_goal_heading))
        goal_msg.max_translation_speed = float(max_translation_speed)
        goal_msg.max_rotation_speed = float(max_rotation_speed)
        return goal_msg

    def rotate_angle(self, angle=1.57, max_rotation_speed=1.9):
        """
        Send a RotateAngle action to the robot.

        Args:
            angle (float): Relative angle (radians) to rotate from the current position.
            max_rotation_speed (float): Maximum rotation speed (rad/s).
        """
        if self.node.enable_security:
            # Encrypt the action name and goal
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("RotateAngle", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac(f"{angle},{max_rotation_speed}", timestamp)
        else:
            # Create the RotateAngle goal message
            goal_msg = RotateAngle.Goal()
            goal_msg.angle = float(angle)
            goal_msg.max_rotation_speed = float(max_rotation_speed)

        # Send the goal
        self._send_goal("RotateAngle", goal_msg)
    
    def parse_rotate_angle_goal(self, goal_str):
        """
        Parse the goal string for the RotateAngle action.

        Args:
            goal_str (str): The goal string in the format "angle,max_rotation_speed".

        Returns:
            RotateAngle.Goal: The constructed goal message.
        """
        angle, max_rotation_speed = map(float, goal_str.split(","))
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed
        return goal_msg
    
    def wall_follow(self, follow_side=1, max_runtime_seconds=5):
        """
        Send a WallFollow action to the robot.

        Args:
            follow_side (int): Side to follow (-1 for right, 1 for left).
            max_runtime_seconds (int): Maximum runtime for the wall-following action in seconds.
        """
        if self.node.enable_security:
            # Encrypt the action name and goal
            timestamp = time.time()
            goal_msg = EncryptedAction.Goal()
            goal_msg.action_name = self.node.encrypt_and_gmac("WallFollow", timestamp)
            goal_msg.encrypted_goal = self.node.encrypt_and_gmac(f"{follow_side},{max_runtime_seconds}", timestamp)
        else:
            # Create the WallFollow goal message
            goal_msg = WallFollow.Goal()
            goal_msg.follow_side = int(follow_side)
            goal_msg.max_runtime.sec = int(max_runtime_seconds)
            goal_msg.max_runtime.nanosec = 0

        # Send the goal
        self._send_goal("WallFollow", goal_msg)
    
    def parse_wall_follow_goal(self, goal_str):
        """
        Parse the goal string for the WallFollow action.

        Args:
            goal_str (str): The goal string in the format "follow_side,max_runtime_seconds".

        Returns:
            WallFollow.Goal: The constructed goal message.
        """
        follow_side, max_runtime_seconds = map(int, goal_str.split(","))
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
        goal_msg.max_runtime.sec = max_runtime_seconds
        goal_msg.max_runtime.nanosec = 0
        return goal_msg

    def parse_goal(self, action_name, goal_str):
        """
        Parse the goal string for the specified action.
        Args:
            action_name (str): The name of the action (e.g., "Dock", "DriveDistance").
            goal_str (str): The goal string to parse.
        Returns:
            Goal: The constructed goal message for the action.
        """
        parsers = {
            "Dock": self.parse_dock_goal,
            "Undock": self.parse_undock_goal,
            "DriveDistance": self.parse_drive_distance_goal,
            "DriveArc": self.parse_drive_arc_goal,
            "RotateAngle": self.parse_rotate_angle_goal,
            "NavigateToPose": self.parse_navigate_to_pose_goal,
        }

        return parsers[action_name](goal_str)