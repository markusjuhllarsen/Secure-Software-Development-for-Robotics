from ssdr.utils.ros_setup import setup_ros2_environment

setup_ros2_environment()
import rclpy
from ssdr.robot_security import SecurityNode

from cryptography.hazmat.primitives.ciphers.aead import AESGCM

def main(args=None):
    rclpy.init(args=args)
    security_node = SecurityNode()

    try:
        # Wait for key exchange to complete
        print("Waiting for key exchange to complete...")
        while security_node.aes_key is None:
            rclpy.spin_once(security_node, timeout_sec=0.1)  # Spin the node to process callbacks
        print("Key exchange completed. AES key derived.")

        # Set up AES-GCM with the derived AES key
        security_node.aesgcm = AESGCM(security_node.aes_key)

        rclpy.spin(security_node)
    except KeyboardInterrupt:
        pass
    finally:
        security_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()