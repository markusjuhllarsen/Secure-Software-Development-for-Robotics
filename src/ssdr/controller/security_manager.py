#!/usr/bin/env python3
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import serialization
from datetime import datetime, timedelta
from utils.config import MAX_ANGULAR_VELOCITY, MAX_LINEAR_VELOCITY

class SecurityManager:
    """Handles security-related functionality for the robot controller"""
    
    def __init__(self, logger):
        self.logger = logger
        
        self.private_key = ec.generate_private_key(ec.SECP256R1())
        self.public_key = self.private_key.public_key()
        
        # Serialize the public key for sharing
        self.public_key_bytes = self.public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        )
        
        self.aes_key = None
        self.peer_ack_received = False

        self.last_command_time = datetime.now()
        self.command_count = 0
        self.max_commands_per_minute = 30  # Rate limiting
    
    def set_gui(self, gui):
        """Set the GUI reference after initialization."""
        self.gui = gui

    def rate_limit_and_sanitize_command(self, linear, angular):
        """
        Validate commands based on rate limiting and sanitization
        
        Args:False
            command_type (str): Type of command being validated (e.g., "move:0.2:0.0")
            
        Returns:
            bool: True if the command is valid, False otherwise
        """
        if self.sanitize_command(linear, angular):
            if not self.rate_limited():
                return True
            else:
                self.logger.info(f"Rate limited: {linear}, {angular}")
                self.gui.update_status(f"Rate limited: {linear}, {angular}")
                return False
        self.logger.info(f"Invalid command: {linear}, {angular}")
        self.gui.update_status(f"Invalid command: {linear}, {angular}")
        return False

    def rate_limited(self):
        """
        Check if the command rate limit has been exceeded
        Returns:
            bool: True if the command is within limits, False otherwise
        """
        current_time = datetime.now()
        
        # Rate limiting check
        time_diff = current_time - self.last_command_timesa
        if time_diff <= timedelta(minutes=1):
            self.command_count += 1
            if self.command_count > self.max_commands_per_minute:
                self.logger.info("Rate limit exceeded")
                self.gui.update_status("Rate limit exceeded")
                return True
        else:
            # Reset count after a minute
            self.last_command_time = current_time
            self.command_count = 1
        return False
    
    def sanitize_command(self, linear, angular):
        """
        Sanitize numerical input to ensure it's within safe bounds
        
        Args:
            input_value: Value to sanitize
            linear: Linear velocity
            angular: Angular velocity
            default: Default value if input is invalid
            
        Returns:
            float: Sanitized value
        """
        try:
            linear = float(linear)
            angular = float(angular)
            lin_sanitized = abs(linear) <= MAX_LINEAR_VELOCITY
            ang_sanitized = abs(angular) <= MAX_ANGULAR_VELOCITY
            if lin_sanitized and ang_sanitized and linear*angular == 0:
                return True
            elif not lin_sanitized:
                self.logger.info(f"Invalid linear velocity: {linear}")
                self.gui.update_status(f"Invalid linear velocity: {linear}")
            elif not ang_sanitized:
                self.logger.info(f"Invalid angular velocity: {angular}")
                self.gui.update_status(f"Invalid angular velocity: {angular}")
            elif linear*angular != 0:
                self.logger.info(f"Invalid command: linear={linear}, angular={angular}")
                self.gui.update_status(f"Invalid command: linear={linear}, angular={angular}")
            return False
        except TypeError:
            return False
    