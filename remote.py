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

    simulation = messagebox.askyesno("Simulation", "Is this the simulation PC?")

    # Prompt the user for the master and host IP addresses
    simulation_ip = simpledialog.askstring("Simulation IP", "Enter the IP address of the simulation PC:")
    if not simulation_ip:
        messagebox.showerror("Error", "Simulation IP is required.")
        return

    if not simulation:
        controller_ip = simpledialog.askstring("Controller IP", "Enter the IP address of the controller PC:")
        if not controller_ip:
            messagebox.showerror("Error", "Controller IP is required.")
            return

    # Set up the ROS environment variables
    try:
        os.environ["ROS_MASTER_URI"] = f"http://{simulation_ip}:11311"
        subprocess.run(f"export ROS_MASTER_URI=http://{simulation_ip}:11311", shell=True, check=True)

        if not simulation:
            os.environ["ROS_IP"] = controller_ip
            subprocess.run(f"export ROS_IP={controller_ip}", shell=True, check=True)
        else:
            os.environ["ROS_IP"] = simulation_ip
            subprocess.run(f"export ROS_IP={simulation_ip}", shell=True, check=True)
            subprocess.run("roscore", shell=True, check=True)

    except Exception as e:
        messagebox.showerror("Error", f"Failed to configure remote connection: {e}")

# Call the function before starting the main application
configure_remote_connection()