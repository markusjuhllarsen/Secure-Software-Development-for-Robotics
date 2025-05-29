#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
import threading
import time
import rclpy

from utils.config import DEFAULT_LINEAR_VELOCITY, DEFAULT_ANGULAR_VELOCITY, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY
from controller.sanitizer_manager import Sanitizer

class ButtonControlGUI:
    """Main GUI class for the Turtlebot4 controller"""
    
    def __init__(
            self, 
            controller_node: rclpy.node
            ) -> None:
        """
        Initialize the GUI
        args:
            controller_node: The ROS2 node that controls the robot
        """
        self.controller = controller_node

        # Create main window
        self.root = tk.Tk()
        self.root.title("Secure Turtlebot4 Controller")
        self.root.geometry("700x700")  # Increased size for better visibility
        self.root.resizable(True, True)  # Allow resizing
        
        # Apply styling
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Arial', 12))
        self.style.configure('TFrame', background='#f0f0f0')
        self.style.configure('TLabel', font=('Arial', 12), background='#f0f0f0')
        self.style.configure('Cancel.TButton', background='red', foreground='white', font=('Arial', 12, 'bold'))
        
        self.create_widgets()
        
        # Start ROS2 spin in a separate thread
        self.spin_thread = threading.Thread(target=self.ros_spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
    
    def create_widgets(
            self
            ) -> None:
        """Create and configure all GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)
    
        # Security tag
        security_status = "SECURE" if self.controller.enable_security else "NOT SECURE"
        security_color = "green" if self.controller.enable_security else "red"
        self.security_label = tk.Label(
            main_frame,
            text=security_status,
            font=('Arial', 12, 'bold'),
            bg=security_color,
            fg="white",
            padx=10,
            pady=5
        )
        self.security_label.grid(row=0, column=0, sticky="w", padx=5, pady=5)

        # Title
        title_label = ttk.Label(main_frame, text="Turtlebot4 Secure Control Panel", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=10)
        
        # Create movement controls
        self._create_movement_controls(main_frame)
        
        # Create docking controls
        self._create_action_controls(main_frame)
        
        # Create status display
        self._create_status_display(main_frame)
        
        # Make all the frames resize properly with window resizing
        for i in range(2):
            main_frame.columnconfigure(i, weight=1)
        for i in range(3):
            main_frame.rowconfigure(i, weight=1)
    
    def _create_movement_controls(
            self, 
            parent: ttk.Frame
            ) -> None:
        """
        Create the movement control buttons
        args:
            parent: The parent frame to place the controls in.
        """
        control_frame = ttk.LabelFrame(parent, text="Movement Controls", padding="15")
        control_frame.grid(row=1, column=0, padx=15, pady=15, sticky="nsew")
        
        self.movement_buttons = {}

        # Movement buttons - increased size and padding
        btn_forward = ttk.Button(control_frame, text="Forward", 
                                command=lambda: self.controller.start_repeating_command(DEFAULT_LINEAR_VELOCITY, 0.0))
        btn_forward.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
        
        self.movement_buttons['forward'] = btn_forward

        btn_left = ttk.Button(control_frame, text="Turn Left", 
                             command=lambda: self.controller.start_repeating_command(0.0, DEFAULT_ANGULAR_VELOCITY))
        btn_left.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        self.movement_buttons['left'] = btn_left

        btn_stop = ttk.Button(control_frame, text="STOP", 
                             command=lambda: self.controller.stop_repeating_command(),
                             style='Stop.TButton')
        self.style.configure('Stop.TButton', background='red', foreground='white', font=('Arial', 12, 'bold'))
        btn_stop.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
        
        self.movement_buttons['stop'] = btn_stop

        btn_right = ttk.Button(control_frame, text="Turn Right", 
                              command=lambda: self.controller.start_repeating_command(0.0, -DEFAULT_ANGULAR_VELOCITY))
        btn_right.grid(row=1, column=2, padx=10, pady=10, sticky="ew")
        
        self.movement_buttons['right'] = btn_right

        btn_backward = ttk.Button(control_frame, text="Backward", 
                                 command=lambda: self.controller.start_repeating_command(-DEFAULT_LINEAR_VELOCITY, 0.0))
        btn_backward.grid(row=2, column=1, padx=10, pady=10, sticky="ew")
        
        self.movement_buttons['backward'] = btn_backward
        # Configure the grid to expand
        for i in range(3):
            control_frame.columnconfigure(i, weight=1)
        for i in range(3):
            control_frame.rowconfigure(i, weight=1)

    def _create_action_controls(
            self, 
            parent: ttk.Frame
            ) -> None:
        """
        Create the docking control buttons
        args:
            parent: The parent frame to place the controls in.
        """
        dock_frame = ttk.LabelFrame(parent, text="Actions", padding="15")
        dock_frame.grid(row=1, column=1, padx=15, pady=15, sticky="nsew")
        
        self.action_buttons = {}

        for i, action in enumerate(self.controller.action_manager.actions):
            btn = ttk.Button(
                dock_frame, 
                text=action,
                command=lambda action=action: self._toggle_action(action))
            btn.grid(row=i, column=0, padx=10, pady=2, sticky="ew")
            self.action_buttons[action] = btn
        
        # Configure the grid to expand
        dock_frame.columnconfigure(0, weight=1)
        for i in range(len(self.action_buttons)):
            dock_frame.rowconfigure(i, weight=1)
    
    def update_button(
            self, 
            action_name: str, 
            state: str
            ) -> None:
        """
        Update the button state in the GUI and make other buttons non-responsive.
        args:
            action_name: The name of the action.
            state: The new state of the button ("Action" or "Cancel").
        """
        button = self.action_buttons[action_name]
        if state == "Cancel":
            button.config(text=f"Cancel {action_name}", style='Cancel.TButton')
            # Disable all buttons
            for other_action, other_button in self.action_buttons.items():
                if other_action != action_name:
                    other_button.config(state=tk.DISABLED)
            for movement_button in self.movement_buttons.values():
                movement_button.config(state=tk.DISABLED)
        else:
            button.config(text=action_name, style='TButton')
            # Re-enable all buttons
            for button in self.action_buttons.values():
                button.config(state=tk.NORMAL)
            for movement_button in self.movement_buttons.values():
                movement_button.config(state=tk.NORMAL)

    def _toggle_action(
            self, 
            action: str
            ) -> None:
        """
        Toggle the action button state and execute the action.
        args:
            action: The name of the action to toggle.
        """
        if action in self.controller.action_manager.active_goals:
            # Cancel action and change button
            self.controller.action_manager.cancel_action(action)
        else:
            # Actions without parameters
            if action in ["Dock", "Undock"]:
                if not Sanitizer.check_rate_limit('action'):
                    messagebox.showerror("Rate Limit Exceeded", 
                                        "Please wait before sending another action command.")
                    return
                getattr(self.controller.action_manager, f"{action.lower()}_robot")()
            elif action in ["DriveArc", "DriveDistance", "NavigateToPosition", "RotateAngle", "WallFollow"]:
                # Actions with parameters
                self._show_parameter_popup(action)

    def _show_parameter_popup(
            self, 
            action: str
            ) -> None:
        """
        Show a popup window to enter parameters for the selected action.
        args:
            action: The name of the action.
        """
        popup = tk.Toplevel(self.root)
        popup.title(f"Enter Parameters for {action}")
        popup.geometry("400x300")
        
        # Parameter fields
        param_labels = []
        param_entries = []

        # Define parameters for each action with their types and limits
        action_params = {
            "DriveArc": [
                {"name": "Translate Direction (1/-1)", "default": 1, "type": "enum", 'enum': [-1,1]},
                {"name": "Angle (radians)", "default": 2.57, "type": "float", "min": None, "max": None},
                {"name": "Radius (meters)", "default": 1.0, "type": "float", "min": 0.0, "max": None},
                {"name": "Max Translation Speed (m/s)", "default": DEFAULT_LINEAR_VELOCITY, "type": "float", "min": 0.0, "max": MAX_LINEAR_VELOCITY}
            ],
            "DriveDistance": [
                {"name": "Distance (meters)", "default": 1.0, "type": "float", "min": None, "max": None},
                {"name": "Max Translation Speed (m/s)", "default": DEFAULT_LINEAR_VELOCITY, "type": "float", "min": 0.0, "max": MAX_LINEAR_VELOCITY}
            ],
            "NavigateToPosition": [
                {"name": "X (meters)", "default": 0.0, "type": "float", "min": None, "max": None},
                {"name": "Y (meters)", "default": 0.0, "type": "float", "min": None, "max": None},
                {"name": "Theta (radians)", "default": 0.0, "type": "float", "min": None, "max": None},
                {"name": "Achieve Goal Heading (1/0)", "default": 1, "type": "bool"},
                {"name": "Max Translation Speed (m/s)", "default": DEFAULT_LINEAR_VELOCITY, "type": "float", "min": 0.0, "max": MAX_LINEAR_VELOCITY},
                {"name": "Max Rotation Speed (rad/s)", "default": DEFAULT_ANGULAR_VELOCITY, "type": "float", "min": 0.0, "max": MAX_ANGULAR_VELOCITY}
            ],
            "RotateAngle": [
                {"name": "Angle (radians)", "default": 1.57, "type": "float", "min": None, "max": None},
                {"name": "Max Rotation Speed (rad/s)", "default": DEFAULT_ANGULAR_VELOCITY, "type": "float", "min": 0.0, "max": MAX_ANGULAR_VELOCITY}
            ],
            "WallFollow": [
                {"name": "Follow Side (1/-1)", "default": 1, "type": "enum", "enum": [-1,1]},
                {"name": "Max Runtime (seconds)", "default": 60, "type": "int", "min": 0.0, "max": None}
            ]
        }

        # Create input fields for the parameters
        for i, param_info in enumerate(action_params[action]):
            label = ttk.Label(popup, text=param_info["name"])
            label.grid(row=i, column=0, padx=10, pady=5, sticky="w")
            entry = ttk.Entry(popup)
            entry.insert(0, param_info["default"])
            entry.grid(row=i, column=1, padx=10, pady=5, sticky="ew")
            param_labels.append(label)
            param_entries.append((entry, param_info))

        # Submit button
        def submit_parameters():
            sanitized_params = []
            
            try:
                # Check action rate limiting first
                if not Sanitizer.check_rate_limit('action'):
                    messagebox.showerror("Rate Limit Exceeded", 
                                        "Please wait before sending another action command.")
                    return
                    
                # Validate and sanitize all parameters
                for entry, param_info in param_entries:
                    value = entry.get()
                    param_name = param_info["name"]
                    param_type = param_info["type"]
                    if param_type == "enum":
                        allowed_values = param_info["enum"]
                        sanitized_value = Sanitizer.sanitize_action_param(
                        value, param_type, None, None, param_name, allowed_values
                    )
                    else:
                        min_val = param_info.get("min")
                        max_val = param_info.get("max")
                        # Sanitize the parameter
                        sanitized_value = Sanitizer.sanitize_action_param(
                            value, param_type, min_val, max_val, param_name
                        )
                    sanitized_params.append(sanitized_value)
                
                # After validation, call the appropriate action
                if action == "DriveArc":
                    self.controller.action_manager.drive_arc(
                        translate_direction=sanitized_params[0],
                        angle=sanitized_params[1],
                        radius=sanitized_params[2],
                        max_translation_speed=sanitized_params[3]
                    )
                elif action == "DriveDistance":
                    self.controller.action_manager.drive_distance(
                        distance=sanitized_params[0],
                        velocity=sanitized_params[1]
                    )
                elif action == "NavigateToPosition":
                    self.controller.action_manager.navigate_to_position(
                        x=sanitized_params[0],
                        y=sanitized_params[1],
                        theta=sanitized_params[2],
                        achieve_goal_heading=sanitized_params[3],
                        max_translation_speed=sanitized_params[4],
                        max_rotation_speed=sanitized_params[5]
                    )
                elif action == "RotateAngle":
                    self.controller.action_manager.rotate_angle(
                        angle=sanitized_params[0],
                        max_rotation_speed=sanitized_params[1]
                    )
                elif action == "WallFollow":
                    self.controller.action_manager.wall_follow(
                        follow_side=sanitized_params[0],
                        max_runtime_seconds=sanitized_params[1]
                    )
                popup.destroy()  # Close the popup after submitting
                
                # Update status
                self.update_status(f"Action {action} started with validated parameters")
            except ValueError as e:
                messagebox.showerror("Invalid Input", f"Error: {e}")
                self.update_status(f"Error in {action} parameters: {e}")

        submit_button = ttk.Button(popup, text="Submit", command=submit_parameters)
        submit_button.grid(row=len(action_params[action]), column=0, columnspan=2, pady=10)

        # Configure the popup grid
        for i in range(len(action_params[action]) + 1):
            popup.rowconfigure(i, weight=1)
        popup.columnconfigure(1, weight=1)
    
    def _create_status_display(
            self, 
            parent: ttk.Frame
            ) -> None:
        """
        Create the status display area
        args:
            parent: The parent frame to place the status display in.
        """
        status_frame = ttk.LabelFrame(parent, text="Status", padding="15")
        status_frame.grid(row=2, column=0, columnspan=2, padx=15, pady=15, sticky="nsew")
        
        self.status_text = tk.Text(status_frame, height=8, width=35, wrap=tk.WORD)  # Increased height and width
        self.status_text.pack(fill=tk.BOTH, expand=True)
        self.status_text.config(state=tk.DISABLED)            
    
    def update_status(
            self, 
            status_msg: str
            ) -> None:
        """
        Update the status display with a new message
        args:
            status_msg: The message to display in the status area.
        """
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] {status_msg}\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
    
    def ros_spin(
            self
            ) -> None:
        """Spin the ROS2 node in a separate thread"""
        while True:
            try:
                rclpy.spin_once(self.controller, timeout_sec=0.1)
                time.sleep(0.01)
            except Exception as e:
                print(f"Error in ROS2 spin: {e}")
                time.sleep(0.1)
    
    def run(
            self
            ) -> None:
        """Start the GUI main loop"""
        self.root.mainloop()