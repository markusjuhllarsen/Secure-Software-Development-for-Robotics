import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import argparse
#from some_msgs.action import MoveRobot  # Replace with the actual action definition
from nav2_msgs.action import Undock  # TurtleBot4 uses nav2_msgs for undocking

class GazeboCommandCLI(Node):
    def __init__(self):
        super().__init__('gazebo_command_cli')
        #self.action_client = ActionClient(self, MoveRobot, '/move_robot')  # Replace '/move_robot' with your action name
        self.undock_action_client = ActionClient(self, Undock, '/undock')  # TurtleBot4 undock action

    def send_undock_action(self):
        if not self.undock_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available for Undock!')
            return

        goal_msg = Undock.Goal()  # Create an empty goal message for the Undock action

        self.get_logger().info('Sending undock action goal...')
        future = self.undock_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected by the action server.')
            return

        self.get_logger().info('Undock goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f'Undock action result: {result.result}')

def main():
    rclpy.init()

    parser = argparse.ArgumentParser(description="CLI for sending ROS2 actions to a TurtleBot4 simulation.")
    parser.add_argument('--linear-x', type=float, default=0.0, help="Linear velocity in the x direction.")
    parser.add_argument('--angular-z', type=float, default=0.0, help="Angular velocity around the z axis.")
    parser.add_argument('--undock', action='store_true', help="Send an undock action.")
    args = parser.parse_args()

    node = GazeboCommandCLI()

    try:
        if args.undock:
            node.send_undock_action()
        else:
            node.send_velocity_action(args.linear_x, args.angular_z)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()