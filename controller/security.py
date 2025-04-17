#!/usr/bin/env python3

import hashlib
import hmac
import secrets
from datetime import datetime, timedelta

class SecurityManager:
    """Handles security-related functionality for the robot controller"""
    
    def __init__(self):
        # Security parameters
        self.secret_key = secrets.token_bytes(32)  # Generate a new key each time for this session
        self.last_command_time = datetime.now()
        self.command_count = 0
        self.max_commands_per_minute = 30  # Rate limiting
    
    def validate_command(self, command_type):
        """
        Validate commands based on various security rules
        
        Args:
            command_type (str): Type of command being validated (e.g., "move:0.2:0.0")
            
        Returns:
            bool: True if the command is valid, False otherwise
        """
        current_time = datetime.now()
        
        # Rate limiting check
        time_diff = current_time - self.last_command_time
        if time_diff <= timedelta(minutes=1):
            self.command_count += 1
            if self.command_count > self.max_commands_per_minute:
                print('Command rate limit exceeded')
                return False
        else:
            # Reset count after a minute
            self.last_command_time = current_time
            self.command_count = 1
        
        # Generate command signature for integrity
        timestamp = current_time.isoformat()
        signature = hmac.new(
            self.secret_key,
            f"{command_type}:{timestamp}".encode(),
            hashlib.sha256
        ).hexdigest()
        
        # Log the secure command
        print(f'Command: {command_type}, Timestamp: {timestamp}, Signature: {signature[:10]}...')
        return True
    
    def sanitize_input(self, input_value, min_value, max_value, default=0.0):
        """
        Sanitize numerical input to ensure it's within safe bounds
        
        Args:
            input_value: Value to sanitize
            min_value: Minimum allowed value
            max_value: Maximum allowed value
            default: Default value if input is invalid
            
        Returns:
            float: Sanitized value
        """
        try:
            value = float(input_value)
            if value < min_value:
                return min_value
            elif value > max_value:
                return max_value
            return value
        except (ValueError, TypeError):
            return default