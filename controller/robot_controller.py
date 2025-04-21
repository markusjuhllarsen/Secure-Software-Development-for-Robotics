#!/usr/bin/env python3

from datetime import datetime
import base64

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

from controller.security_manager import SecurityManager
from controller.actions import RobotActionManager
from utils.security import encrypt_and_gmac, exchange_keys

class SecureTurtlebot4Controller(Node):
    """Main controller class for the Turtlebot4 with security features"""
    
    def __init__(self, encrypt=False):
        super().__init__('secure_turtlebot4_controller')
        
        # Print node and topic info at startup
        self.get_logger().info("Starting Secure Turtlebot4 Controller - if controls don't work, check topic names")
        
        # Reference to GUI (will be set in main.py)
        self.gui = None

        # Use reentrant callback group for actions
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize security manager
        self.security = SecurityManager(self.get_logger())
        
        self.encrypt = encrypt
        # Initialize publishers
        self.publisher_list = []
        self._init_publishers()

        if self.encrypt:
            self.public_key_timer = self.create_timer(1, self.publish_public_key)

        self._init_subscribers()

        # Initialize docking manager
        self.action_manager = RobotActionManager(self, enable_security=self.encrypt)
        
        self.get_logger().info('Secure Turtlebot4 Controller initialized')
    
    def _init_publishers(self):
        """Initialize all publishers"""
        if self.encrypt:
            self.get_logger().info("Encryption enabled for cmd_vel messages")
            self.publisher_list.append(self.create_publisher(
                String,
                'encrypted_topic',  # Encrypted topic for cmd_vel
                10
            ))

            self.public_key_publisher = self.create_publisher(
                String,
                'public_key_controller',  # Topic for sharing public key
                10
            )

            self.aes_key_ack_publisher = self.create_publisher(
                String, 
                '/aes_key_ack', 
                10
            )

        else:
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
            
            # Status publisher/subscriber for monitoring
            self.status_publisher = self.create_publisher(
                String,
                '/turtlebot4/status_msg',  # Status topic
                10
            )
        
        #self.status_subscriber = self.create_subscription(
        #    String,
        #    '/turtlebot4/status_msg', 
        #    self.status_callback,
        #    10
        #)
    
    def _init_subscribers(self):
        if self.encrypt:
            self.public_key_subscription = self.create_subscription(
                String, 
                '/public_key_robot', 
                self.receive_public_key_callback, 
                10
            )

            self.create_subscription(
                String, 
                '/aes_key_ack', 
                self.receive_aes_key_ack_callback, 
                10
            )

    def send_aes_key_ack(self):
        msg = String()
        msg.data = f"{self.security.public_key_bytes}"  # Include the node ID in the ACK
        self.aes_key_ack_publisher.publish(msg)
        self.get_logger().info("Sent acknowledgment to peer.")
    
    def receive_aes_key_ack_callback(self, msg):
        try:
            peer_public_key = msg.data # Split the ACK message into parts
            if peer_public_key != self.security.public_key_bytes:
                self.security.peer_ack_received = True
                self.get_logger().info(f"Received acknowledgment from peer")
        except Exception as e:
            self.get_logger().error(f"Failed to process acknowledgment: {e}")

    def publish_public_key(self):
        if self.security.peer_ack_received:
            # Stop publishing once the AES key is established
            self.public_key_timer.cancel()
            self.get_logger().info("Peer AES key established. Stopped publishing public key.")
            return

        # Publish the public key
        msg = String()
        msg.data = self.security.public_key_bytes.decode('utf-8')  # Convert bytes to string
        self.public_key_publisher.publish(msg)
        self.get_logger().info("Published public key.")

    def receive_public_key_callback(self, msg):
        try:
            # Deserialize the peer's public key
            peer_public_key_bytes = msg.data.encode('utf-8')  # Convert string back to bytes
            self.security.aes_key = exchange_keys(self.security.private_key, peer_public_key_bytes)
            print(self.security.aes_key)
            self.send_aes_key_ack()
            self.get_logger().info("Shared AES key derived successfully.")

            # Stop subscribing to the public key topic
            self.destroy_subscription(self.public_key_subscription)
            self.get_logger().info("Unsubscribed from /public_key topic.")
        except Exception as e:
            self.get_logger().error(f"Failed to derive AES key: {e}")
    
    def status_callback(self, msg):
        """
        Callback for status messages from the robot
        """
        # Log status messages from the robot
        self.get_logger().info(msg.data)
        
        # Update GUI with the status message
        if hasattr(self, 'gui') and self.gui:
            self.gui.update_status("msg.data")
    
    def move_robot(self, linear_x, angular_z):
        """
        Send movement command with specified linear and angular velocities
        Publishing to all cmd_vel topics to ensure it works
        """
        # Security validation
        if not self.security.rate_limit_and_sanitize_command(linear_x, angular_z):
            return False
        
        linear_x = float(linear_x)
        angular_z = float(angular_z)

        if self.encrypt:
            timestamp = datetime.now().isoformat()
            message = f"{linear_x},{angular_z}"
            nonce, encrypted = encrypt_and_gmac(self.security.aesgcm, message, timestamp)

            nonce_b64 = base64.b64encode(nonce).decode()
            encrypted_data_b64 = base64.b64encode(encrypted).decode()
            encrypted_msg = f"{nonce_b64}|{encrypted_data_b64}"
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
        #else:
        #    status = f"Moving: linear={linear_x}, angular={angular_z}" #!!!!
            
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