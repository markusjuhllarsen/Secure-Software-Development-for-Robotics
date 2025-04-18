#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
import threading
import time
import rclpy

from utils.config import DEFAULT_LINEAR_VELOCITY, DEFAULT_ANGULAR_VELOCITY, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY

class ButtonControlGUI:
    """Main GUI class for the Turtlebot4 controller"""
    
    def __init__(self, controller_node):
        """
        Initialize the GUI
        
        Args:
            controller_node: The ROS2 node that controls the robot
        """
        self.controller = controller_node
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Secure Turtlebot4 Controller")
        self.root.geometry("700x600")  # Increased size for better visibility
        self.root.resizable(True, True)  # Allow resizing
        
        # Apply styling
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Arial', 12))
        self.style.configure('TFrame', background='#f0f0f0')
        self.style.configure('TLabel', font=('Arial', 12), background='#f0f0f0')
        
        self.create_widgets()
        
        # Start ROS2 spin in a separate thread
        self.spin_thread = threading.Thread(target=self.ros_spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
    
    def create_widgets(self):
        """Create and configure all GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="Turtlebot4 Secure Control Panel", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=10)
        
        # Create movement controls
        self._create_movement_controls(main_frame)
        
        # Create docking controls
        self._create_docking_controls(main_frame)
        
        # Create custom velocity controls
        self._create_velocity_controls(main_frame)
        
        # Create status display
        self._create_status_display(main_frame)
        
        # Add keyboard shortcuts
        self._setup_keyboard_shortcuts()
        
        # Make all the frames resize properly with window resizing
        for i in range(2):
            main_frame.columnconfigure(i, weight=1)
        for i in range(3):
            main_frame.rowconfigure(i, weight=1)
    
    def _create_movement_controls(self, parent):
        """Create the movement control buttons"""
        control_frame = ttk.LabelFrame(parent, text="Movement Controls", padding="15")
        control_frame.grid(row=1, column=0, padx=15, pady=15, sticky="nsew")
        
        # Movement buttons - increased size and padding
        btn_forward = ttk.Button(control_frame, text="Forward", 
                                command=lambda: self.controller.move_robot(DEFAULT_LINEAR_VELOCITY, 0.0))
        btn_forward.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
        
        btn_left = ttk.Button(control_frame, text="Turn Left", 
                             command=lambda: self.controller.move_robot(0.0, DEFAULT_ANGULAR_VELOCITY))
        btn_left.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        btn_stop = ttk.Button(control_frame, text="STOP", 
                             command=lambda: self.controller.move_robot(0.0, 0.0),
                             style='Stop.TButton')
        self.style.configure('Stop.TButton', background='red', foreground='white', font=('Arial', 12, 'bold'))
        btn_stop.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
        
        btn_right = ttk.Button(control_frame, text="Turn Right", 
                              command=lambda: self.controller.move_robot(0.0, -DEFAULT_ANGULAR_VELOCITY))
        btn_right.grid(row=1, column=2, padx=10, pady=10, sticky="ew")
        
        btn_backward = ttk.Button(control_frame, text="Backward", 
                                 command=lambda: self.controller.move_robot(-DEFAULT_LINEAR_VELOCITY, 0.0))
        btn_backward.grid(row=2, column=1, padx=10, pady=10, sticky="ew")
        
        # Configure the grid to expand
        for i in range(3):
            control_frame.columnconfigure(i, weight=1)
        for i in range(3):
            control_frame.rowconfigure(i, weight=1)
    
    def _create_docking_controls(self, parent):
        """Create the docking control buttons"""
        dock_frame = ttk.LabelFrame(parent, text="Docking Controls", padding="15")
        dock_frame.grid(row=1, column=1, padx=15, pady=15, sticky="nsew")
        
        # Add Dock and Undock buttons with increased size and padding
        btn_dock = ttk.Button(dock_frame, text="Dock Robot", 
                             command=self.controller.docking.dock_robot)
        btn_dock.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        btn_undock = ttk.Button(dock_frame, text="Undock Robot", 
                               command=self.controller.docking.undock_robot)
        btn_undock.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        # Configure the grid to expand
        dock_frame.columnconfigure(0, weight=1)
        for i in range(2):
            dock_frame.rowconfigure(i, weight=1)
    
    def _create_velocity_controls(self, parent):
        """Create controls for custom velocity input"""
        velocity_frame = ttk.LabelFrame(parent, text="Custom Velocity", padding="15")
        velocity_frame.grid(row=2, column=0, padx=15, pady=15, sticky="nsew")
        
        ttk.Label(velocity_frame, text="Linear (m/s):").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.linear_entry = ttk.Entry(velocity_frame, width=12)  # Wider entry field
        self.linear_entry.grid(row=0, column=1, padx=10, pady=10)
        self.linear_entry.insert(0, "0.0")
        
        ttk.Label(velocity_frame, text="Angular (rad/s):").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        self.angular_entry = ttk.Entry(velocity_frame, width=12)  # Wider entry field
        self.angular_entry.grid(row=1, column=1, padx=10, pady=10)
        self.angular_entry.insert(0, "0.0")
        
        btn_send = ttk.Button(velocity_frame, text="Send Command", 
                             command=self.send_custom_velocity)
        btn_send.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        
        # Configure the grid to expand
        for i in range(2):
            velocity_frame.columnconfigure(i, weight=1)
        for i in range(3):
            velocity_frame.rowconfigure(i, weight=1)
    
    def _create_status_display(self, parent):
        """Create the status display area"""
        status_frame = ttk.LabelFrame(parent, text="Status", padding="15")
        status_frame.grid(row=2, column=1, columnspan=1, padx=15, pady=15, sticky="nsew")
        
        self.status_text = tk.Text(status_frame, height=8, width=35, wrap=tk.WORD)  # Increased height and width
        self.status_text.pack(fill=tk.BOTH, expand=True)
        self.status_text.config(state=tk.DISABLED)
    
    def _setup_keyboard_shortcuts(self):
        """Set up keyboard shortcuts for robot control"""
        self.root.bind('<Up>', lambda event: self.controller.move_robot(DEFAULT_LINEAR_VELOCITY, 0.0))
        self.root.bind('<Down>', lambda event: self.controller.move_robot(-DEFAULT_LINEAR_VELOCITY, 0.0))
        self.root.bind('<Left>', lambda event: self.controller.move_robot(0.0, DEFAULT_ANGULAR_VELOCITY))
        self.root.bind('<Right>', lambda event: self.controller.move_robot(0.0, -DEFAULT_ANGULAR_VELOCITY))
        self.root.bind('<space>', lambda event: self.controller.move_robot(0.0, 0.0))
        self.root.bind('d', lambda event: self.controller.docking.dock_robot())
        self.root.bind('u', lambda event: self.controller.docking.undock_robot())
    
    def send_custom_velocity(self):
        """Handle custom velocity input and send to robot"""
        try:
            linear = float(self.linear_entry.get())
            angular = float(self.angular_entry.get())
            
            # Validate input values for safety
            if abs(linear) > MAX_LINEAR_VELOCITY:
                messagebox.showerror("Invalid Input", 
                                    f"Linear velocity must be between -{MAX_LINEAR_VELOCITY} and {MAX_LINEAR_VELOCITY} m/s")
                return
            if abs(angular) > MAX_ANGULAR_VELOCITY:
                messagebox.showerror("Invalid Input", 
                                    f"Angular velocity must be between -{MAX_ANGULAR_VELOCITY} and {MAX_ANGULAR_VELOCITY} rad/s")
                return
                
            self.controller.move_robot(linear, angular)
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for velocities")
    
    def update_status(self, status_msg):
        """Update the status display with a new message"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] {status_msg}\n")
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
    
    def ros_spin(self):
        """Spin the ROS2 node in a separate thread"""
        while True:
            try:
                rclpy.spin_once(self.controller, timeout_sec=0.1)
                time.sleep(0.01)
            except Exception as e:
                print(f"Error in ROS2 spin: {e}")
                time.sleep(0.1)
    
    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()