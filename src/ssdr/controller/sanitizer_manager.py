from typing import Any
import time
from threading import Lock

from config import MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY

class Sanitizer:
    """
    A utility class to sanitize and validate inputs for robot actions.
    """
    # Rate limiting settings
    _last_command_time = {}  # Dict to track last command time for each command type
    _command_intervals = {
        'movement': 2.0,     # Minimum seconds between movement commands in seconds
        'action': 5.0,     # Minimum seconds between action commands
    }
    _rate_limit_lock = Lock()  # Thread safety for rate limiting

    @staticmethod
    def check_rate_limit(
        command_type: str
        )-> bool:
        """
        Check if a command passes rate limiting requirements
        args:
            command_type: Type of command for rate limiting ('movement', 'action', etc.)  
        returns:
            If command is allowed
        """
        with Sanitizer._rate_limit_lock:
            current_time = time.time()
            min_interval = Sanitizer._command_intervals[command_type]
            
            # Get last command time for this type, default to 0 if not set
            last_time = Sanitizer._last_command_time.get(command_type, 0)
            
            # Check if enough time has passed
            if current_time - last_time < min_interval:
                return False
            
            # Update last command time and allow the command
            Sanitizer._last_command_time[command_type] = current_time
            return True

    @staticmethod
    def sanitize_action_param(
        value: Any, 
        param_type: str, 
        min_value: Any, 
        max_value: Any, 
        name: str,
        allowed_values: list[int] = None
        ) -> Any:
        """
        Sanitize an action parameter based on its expected type
        args:
            value: The parameter value to sanitize
            param_type: The expected type ('float', 'int', 'bool', 'enum')
            min_value: Minimum allowed value for numeric types
            max_value: Maximum allowed value for numeric types
            name: Name of the parameter for error messages   
            allowed_values: List of allowed values for enum types
        returns:
            The sanitized parameter value   
        raises:
            ValueError: If the value is invalid or out of range
        """
        if param_type == 'float':
            return Sanitizer.sanitize_float(value, min_value, max_value, name)
        elif param_type == 'int':
            Sanitizer.sanitize_int(value, allowed_values, name)
        elif param_type == 'bool':
            Sanitizer.sanitize_bool(value, name)
        elif param_type == 'enum':
            Sanitizer.sanitize_enum(value, allowed_values, name)
        else:
            raise ValueError(f"Unsupported parameter type: {param_type}. Expected 'float', 'int', or 'bool'.")
    
    @staticmethod
    def sanitize_bool(
            value: Any, 
            name: str
        ) -> bool:
        """
        Sanitize a boolean input by converting to boolean type
        args:
            value: The input value to sanitize.
            name: The name of the parameter (for error messages).
        returns:
            The sanitized boolean value.
        raises:
            ValueError: If the value cannot be converted to a boolean.
        """
        if value in (True, False, 1, 0, '1', '0', 'True', 'False', 'true', 'false'):
            if value in (1, '1', 'True', 'true'):
                return True
            return False
        raise ValueError(f"{name} must be a boolean value (True/False, 1/0).")

    @staticmethod
    def sanitize_enum(
            value: Any, 
            allowed_values: list[int], 
            name: str
        ) -> int:
        """
        Sanitize an enum input by checking against allowed values
        args:
            value: The input value to sanitize.
            allowed_values: List of allowed integer values for the enum.
            name: The name of the parameter (for error messages).
        returns:
            The sanitized value.
        raises:
            ValueError: If the value is not in the allowed values.
        """
        try:
            value = int(float(value))  # Ensure value is an integer
        except ValueError:
            raise ValueError(f"{name} must be a valid integer for enum type.")
        if allowed_values is None:
            raise ValueError(f"{name} must be one of the allowed values: {allowed_values}.")
        if int(value) not in allowed_values:
            raise ValueError(f"{name} must be one of the allowed values: {allowed_values}.")
        return value
    
    @staticmethod
    def sanitize_int(
            value: Any, 
            min_value: int, 
            max_value: int, 
            name: str
        ) -> int:
        """
        Sanitize an integer input by checking against allowed values
        args:
            value: The input value to sanitize.
            min_value: Minimum allowed value
            max_value: Maximum allowed value
            name: The name of the parameter (for error messages).
        returns:
            The sanitized value.
        raises:
            ValueError: If the value is not in the allowed values.
        """
        try:
            int_value = int(float(value))  # Convert through float for robustness
        except ValueError:
            raise ValueError(f"{name} must be a valid integer.")
        if min_value is not None and int_value < min_value:
            raise ValueError(f"{name} must be at least {min_value}.")
        if max_value is not None and int_value > max_value:
            raise ValueError(f"{name} must be at most {max_value}.")
        return int_value

    @staticmethod
    def sanitize_float(
        value: float, 
        min_value: float, 
        max_value: float, 
        name: str
        ) -> float:
        """
        Sanitize a float input by ensuring it is within the specified range
        args:
            value: The input value to sanitize.
            min_value: The minimum allowed value (inclusive).
            max_value: The maximum allowed value (inclusive).
            name: The name of the parameter (for error messages).
        returns:
            The sanitized value.
        raises:
            ValueError: If the value is not within the specified range.
        """
        try:
            value = float(value)
        except ValueError:
            raise ValueError(f"{name} must be a valid float.")

        if min_value is not None and value < min_value:
            raise ValueError(f"{name} must be at least {min_value}.")
        if max_value is not None and value > max_value:
            raise ValueError(f"{name} must be at most {max_value}.")

        return value