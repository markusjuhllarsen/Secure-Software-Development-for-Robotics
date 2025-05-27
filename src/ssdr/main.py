#!/usr/bin/env python3
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from controller.robot_controller import SecureTurtlebot4Controller
from gui.gui_app import ButtonControlGUI
from utils.config import APP_NAME, APP_VERSION

def main():
    """Main entry point of the application"""
    print(f"Starting {APP_NAME} v{APP_VERSION}.")
    
    # Initialize ROS2
    rclpy.init()

    # Auto-detect if the security node is running
    enable_security = False
    
    temp_node = rclpy.create_node('temp_node')  
    rclpy.spin_once(temp_node, timeout_sec=1.0)

    for service_name, _ in temp_node.get_service_names_and_types():
        if service_name == '/security_node/exchange_public_key':
            enable_security = True
            break

    temp_node.destroy_node()
    
    # Create the controller node
    controller_node = SecureTurtlebot4Controller(enable_security)
    
    # Create and run the GUI
    app = ButtonControlGUI(controller_node)
    
    # Set the reference to the GUI in the controller for status updates
    controller_node.gui = app

    controller_node.action_manager.update_button_callback = app.update_button

    if enable_security:
        app.update_status("Waiting for key exchange to complete...")
        with controller_node.key_exchange_condition:
            while controller_node.aes_key is None:
                controller_node.key_exchange_condition.wait()
        controller_node.aesgcm = AESGCM(controller_node.aes_key)
        app.update_status("Key exchange completed successfully.")

    # Set up executor for handling actions properly
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    
    # Start the executor in a separate thread
    executor_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    executor_thread.start()

    # Add initial status message
    controller_node.publish_status("Controller fully initialized. Ready for commands.")
    
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()