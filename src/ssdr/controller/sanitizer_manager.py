from utils.config import MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY
import time
from threading import Lock

class Sanitizer:
    """
    A utility class to sanitize and validate inputs for robot actions.
    """
    # Rate limiting settings
    _last_command_time = {}  # Dict to track last command time for each command type
    _command_intervals = {
        'movement': 5,     # Minimum seconds between movement commands in seconds
        'action': 3.0,       # Minimum seconds between action commands
        'default': 2       # Default minimum interval
    }
    _rate_limit_lock = Lock()  # Thread safety for rate limiting

    @staticmethod
    def check_rate_limit(command_type='default'):
        """
        Check if a command passes rate limiting requirements
        
        Args:
            command_type (str): Type of command for rate limiting ('movement', 'action', etc.)
            
        Returns:
            bool: True if command is allowed, False if it should be rate limited
        """
        with Sanitizer._rate_limit_lock:
            current_time = time.time()
            min_interval = Sanitizer._command_intervals.get(command_type, 
                                                         Sanitizer._command_intervals['default'])
            
            # Get last command time for this type, default to 0 if not set
            last_time = Sanitizer._last_command_time.get(command_type, 0)
            
            # Check if enough time has passed
            if current_time - last_time < min_interval:
                return False
            
            # Update last command time and allow the command
            Sanitizer._last_command_time[command_type] = current_time
            return True

    @staticmethod
    def sanitize_velocity(value, min_value=-1.0, max_value=1.0, name="velocity"):
        """
        Sanitize a velocity input (linear or angular) by ensuring it is within the specified range.

        Args:
            value (float): The velocity to sanitize.
            min_value (float): The minimum allowed velocity.
            max_value (float): The maximum allowed velocity.
            name (str): The name of the parameter (for error messages).

        Returns:
            float: The sanitized velocity.

        Raises:
            ValueError: If the value is not within the specified range.
        """
        # Apply rate limiting for velocity commands
        if not Sanitizer.check_rate_limit('movement'):
            return False
            
        if "linear" in name.lower():
            default_max = MAX_LINEAR_VELOCITY
        elif "angular" in name.lower():
            default_max = MAX_ANGULAR_VELOCITY
        else:
            # Fallback if name doesn't specify type, though ideally it should
            default_max = MAX_LINEAR_VELOCITY # Or handle as an error/warning

        if max_value is None:
            max_value = default_max
        if min_value is None:
            min_value = -default_max

        return Sanitizer.sanitize_float(value, min_value=min_value, max_value=max_value, name=name)

    @staticmethod
    def sanitize_action_param(value, param_type, min_value=None, max_value=None, name="parameter"):
        """
        Sanitize an action parameter based on its expected type
        
        Args:
            value: The parameter value to sanitize
            param_type (str): The expected type ('float', 'int', 'bool')
            min_value: Minimum allowed value for numeric types
            max_value: Maximum allowed value for numeric types
            name (str): Name of the parameter for error messages
            
        Returns:
            The sanitized parameter value
            
        Raises:
            ValueError: If the value is invalid or out of range
        """
        if param_type == 'float':
            return Sanitizer.sanitize_float(value, min_value, max_value, name)
        elif param_type == 'int':
            try:
                int_value = int(float(value))  # Convert through float for robustness
                if min_value is not None and int_value < min_value:
                    raise ValueError(f"{name} must be at least {min_value}.")
                if max_value is not None and int_value > max_value:
                    raise ValueError(f"{name} must be at most {max_value}.")
                return int_value
            except ValueError:
                raise ValueError(f"{name} must be a valid integer.")
        elif param_type == 'bool':
            if value in (True, False, 1, 0, '1', '0', 'True', 'False', 'true', 'false'):
                if value in (1, '1', 'True', 'true'):
                    return True
                return False
            raise ValueError(f"{name} must be a boolean value (True/False, 1/0).")
        else:
            return Sanitizer.sanitize_string(value, name)


    @staticmethod
    def sanitize_status_text(value, name="status_text"):
        """
        Sanitize a status text input by ensuring it is a valid string.

        Args:
            value (str): The status text to sanitize.
            name (str): The name of the parameter (for error messages).

        Returns:
            str: The sanitized status text.

        Raises:
            ValueError: If the value is not a valid string.
        """
        return Sanitizer.sanitize_string(value, name=name)

    @staticmethod
    def sanitize_float(value, min_value=None, max_value=None, name="value"):
        """
        Sanitize a float input by ensuring it is within the specified range.

        Args:
            value (float): The input value to sanitize.
            min_value (float): The minimum allowed value (inclusive).
            max_value (float): The maximum allowed value (inclusive).
            name (str): The name of the parameter (for error messages).

        Returns:
            float: The sanitized value.

        Raises:
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

    @staticmethod
    def sanitize_string(value, name="value"):
        """
        Sanitize a string input by ensuring it is a valid string.

        Args:
            value (str): The input value to sanitize.
            name (str): The name of the parameter (for error messages).

        Returns:
            str: The sanitized value.

        Raises:
            ValueError: If the value is not a valid string.
        """
        if not isinstance(value, str):
            raise ValueError(f"{name} must be a valid string.")
        return value