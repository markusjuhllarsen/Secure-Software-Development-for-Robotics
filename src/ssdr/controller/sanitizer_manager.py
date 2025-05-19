from utils.config import MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY
class Sanitizer:
    """
    A utility class to sanitize and validate inputs for robot actions.
    """

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