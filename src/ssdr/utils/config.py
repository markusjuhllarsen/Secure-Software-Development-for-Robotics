#!/usr/bin/env python3

"""
Configuration constants for the Turtlebot4 controller application
"""

# Application information
APP_NAME = "Secure Turtlebot4 Controller"
APP_VERSION = "1.0.0"

# Robot movement parameters
DEFAULT_LINEAR_VELOCITY = 0.2  # m/s
DEFAULT_ANGULAR_VELOCITY = 0.5  # rad/s
MAX_LINEAR_VELOCITY = 1000  # m/s
MAX_ANGULAR_VELOCITY = 1000  # rad/s

# Security parameters
MAX_COMMANDS_PER_MINUTE = 30
HMAC_ALGORITHM = "sha256"
ENC_ALGORITHM = "aes-256-gcm"

# Status topic
STATUS_TOPIC = '/turtlebot4/status_msg'