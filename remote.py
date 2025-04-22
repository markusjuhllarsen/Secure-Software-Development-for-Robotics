import tkinter as tk
from tkinter import simpledialog, messagebox
import os
import subprocess

def configure_remote_connection():
    """
    Collect IP addresses for master and host, configure the environment, and display commands for the master PC.
    """
    # Create a Tkinter root window (hidden)
    root = tk.Tk()
    root.withdraw()  # Hide the root window

    # Prompt the user for the master and host IP addresses
    master_ip = simpledialog.askstring("Master IP", "Enter the IP address of the master PC:")
    if not master_ip:
        messagebox.showerror("Error", "Master IP is required.")
        return

    host_ip = simpledialog.askstring("Host IP", "Enter the IP address of this host PC:")
    if not host_ip:
        messagebox.showerror("Error", "Host IP is required.")
        return

    # Set up the ROS environment variables
    try:
        os.environ["ROS_MASTER_URI"] = f"http://{master_ip}:11311"
        os.environ["ROS_IP"] = host_ip

        # Execute shell commands to export these variables
        subprocess.run(f"export ROS_MASTER_URI=http://{master_ip}:11311", shell=True, check=True)
        subprocess.run(f"export ROS_IP={host_ip}", shell=True, check=True)

        # Display the commands for the user to run on the master PC
        commands = (
            f"On the master PC, run the following commands:\n\n"
            f"export ROS_MASTER_URI=http://{master_ip}:11311\n"
            f"export ROS_IP={master_ip}\n"
            f"roscore"
        )
        messagebox.showinfo("Master PC Setup", commands)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to configure remote connection: {e}")

# Call the function before starting the main application
configure_remote_connection()