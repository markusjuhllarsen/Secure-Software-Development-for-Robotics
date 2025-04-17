#!/usr/bin/env python3

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