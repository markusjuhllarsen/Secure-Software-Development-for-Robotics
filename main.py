#!/usr/bin/env python3

import os
import sys
import subprocess

def setup_ros2_environment():
    # Source the setup file and restart the script with the updated environment
    ros_setup_file = "/opt/ros/jazzy/setup.bash"
    if "PYTHONPATH" not in os.environ: # Check if the environment variable is set
        if os.path.exists(ros_setup_file):
            # Source the setup file and capture environment variables
            command = f"bash -c 'source {ros_setup_file} && env'"
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, executable="/bin/bash")
            output, _ = proc.communicate()

            # Parse the environment variables
            env_vars = {}
            for line in output.decode("utf-8").splitlines():
                key, _, value = line.partition("=")
                env_vars[key] = value.strip()

            # Restart the script with the updated environment
            os.execvpe(sys.executable, [sys.executable] + sys.argv, {**os.environ, **env_vars})
        else:
            print(f"Error: ROS 2 setup file not found at {ros_setup_file}")
            sys.exit(1)

setup_ros2_environment()

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from controller.robot_controller import SecureTurtlebot4Controller
from gui.gui_app import ButtonControlGUI
from utils.config import APP_NAME, APP_VERSION

def main():
    """Main entry point of the application"""
    print(f"Starting {APP_NAME} v{APP_VERSION}...")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create the controller node
    controller_node = SecureTurtlebot4Controller()
    
    # Set up executor for handling actions properly
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    
    # Start the executor in a separate thread
    executor_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    executor_thread.start()
    
    # Print debug info
    print("ROS2 node initialized. Looking for topics...")
    
    # Create and run the GUI
    app = ButtonControlGUI(controller_node)
    
    # Set the reference to the GUI in the controller for status updates
    controller_node.gui = app
    controller_node.security.set_gui(app)
    
    # Add initial status message
    app.update_status("Controller initialized. Ready for commands.")
    
    try:
        # Add a message to check topics if nothing is happening
        print("GUI started. Use the buttons to control the robot.")
        print("Use 'D' key for docking and 'U' key for undocking.")
        
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()