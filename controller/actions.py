#!/usr/bin/env python3

from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock

class DockingManager:
    """Manages docking and undocking actions for the Turtlebot4"""
    
    def __init__(self, node):
        """
        Initialize the docking manager
        
        Args:
            node: The ROS2 node that will own the action clients
        """
        self.node = node
        
        # Dock and Undock action clients
        self.dock_client = ActionClient(
            node,
            Dock,
            '/dock',
            callback_group=node.callback_group
        )
        
        self.undock_client = ActionClient(
            node,
            Undock,
            '/undock',
            callback_group=node.callback_group
        )
    
    def dock_robot(self):
        """
        Send dock action to robot
        
        Returns:
            bool: True if the action was sent successfully, False otherwise
        """
        # Security validation
        #if not self.node.security.rate_limit_and_sanitize_command("dock"):
        #    return False
            
        self.node.get_logger().info("Sending dock action...")
        
        # Check if action server is available
        if not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Dock action server not available!")
            self.node.publish_status("Dock action server not available!")
            return False
            
        # Create and send goal
        goal_msg = Dock.Goal()
        self._send_dock_goal_future = self.dock_client.send_goal_async(goal_msg)
        self._send_dock_goal_future.add_done_callback(self._dock_goal_response_callback)
        self.node.publish_status("Dock action sent")
        return True
            
    def _dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Dock goal rejected!")
            self.node.publish_status("Dock goal rejected!")
            return
            
        self.node.get_logger().info("Dock goal accepted, waiting for result...")
        self.node.publish_status("Dock goal accepted, waiting for result...")
        
        # Request result
        self._get_dock_result_future = goal_handle.get_result_async()
        self._get_dock_result_future.add_done_callback(self._dock_result_callback)
        
    def _dock_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f"Dock completed with result: {result}")
        self.node.publish_status(f"Dock completed")
    
    def undock_robot(self):
        """
        Send undock action to robot
        
        Returns:
            bool: True if the action was sent successfully, False otherwise
        """
        # Security validation
        #if not self.node.security.rate_limit_and_sanitize_command("undock"):
        #    return False
            
        self.node.get_logger().info("Sending undock action...")
        
        # Check if action server is available
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Undock action server not available!")
            self.node.publish_status("Undock action server not available!")
            return False
            
        # Create and send goal
        goal_msg = Undock.Goal()
        self._send_undock_goal_future = self.undock_client.send_goal_async(goal_msg)
        self._send_undock_goal_future.add_done_callback(self._undock_goal_response_callback)
        self.node.publish_status("Undock action sent")
        return True
            
    def _undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Undock goal rejected!")
            self.node.publish_status("Undock goal rejected!")
            return
            
        self.node.get_logger().info("Undock goal accepted, waiting for result...")
        self.node.publish_status("Undock goal accepted, waiting for result...")
        
        # Request result
        self._get_undock_result_future = goal_handle.get_result_async()
        self._get_undock_result_future.add_done_callback(self._undock_result_callback)
        
    def _undock_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f"Undock completed with result: {result}")
        self.node.publish_status(f"Undock completed")