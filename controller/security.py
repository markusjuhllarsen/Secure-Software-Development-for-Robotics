#!/usr/bin/env python3
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import hashlib
import hmac
import secrets
from datetime import datetime, timedelta
from utils.config import MAX_ANGULAR_VELOCITY, MAX_LINEAR_VELOCITY

class SecurityManager:
    """Handles security-related functionality for the robot controller"""
    
    def __init__(self, logger):
        self.logger = logger
        # Security parameters
        self.secret_key = secrets.token_bytes(32)  # Generate a new key each time for this session
        self.last_command_time = datetime.now()
        self.command_count = 0
        self.max_commands_per_minute = 30  # Rate limiting
        self.aesgcm = AESGCM(self.secret_key)
    
    def set_gui(self, gui):
        """Set the GUI reference after initialization."""
        self.gui = gui

    def rate_limit_and_sanitize_command(self, linear, angular):
        """
        Validate commands based on rate limiting and sanitization
        
        Args:
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
        time_diff = current_time - self.last_command_time
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
        
    def encrypt_and_gmac(self, message):
        """
        Encrypt a message and generate GMAC to ensure confidentiality and integrity
        Args:
            message (str): The command message to generate the MAC for.
    
        Returns:
            str: The GMAC as a hexadecimal string.
        """
        current_time = datetime.now()
        timestamp = current_time.isoformat()

        data = f"{message}:{timestamp}".encode(),

        nonce = secrets.token_bytes(12)
        encrypted_data = self.aesgcm.encrypt(nonce, data, None)

        self.logger.info(f"Message: {message}, Timestamp: {timestamp}, Encrypted Data: {encrypted_data.hex()[:10]}")

        return {
            "encrypted_data": encrypted_data,
            "nonce": nonce,
        }
    
    def decrypt_and_verify(self, encrypted_data, nonce):
        """
        Decrypt the message and verify GMAC to ensure confidentiality and integrity
        Args:
            encrypted_data (bytes): The encrypted command message.
            nonce (bytes): The nonce used for encryption.
    
        Returns:
            str: The decrypted message.
        """
        try:
            # Decrypt the data and verify GMAC
            # The GMAC is verified during decryption, so if it fails, an exception will be raised
            decrypted_data = self.aesgcm.decrypt(nonce, encrypted_data, None)
            return decrypted_data.decode()
        except Exception as e:
            self.logger.error(f"Decryption failed: {e}")
            return None