import os
import sys
import subprocess

def setup_ros2_environment():
    # Source the setup file and restart the script with the updated environment
    ros_setup_file = "/opt/ros/jazzy/setup.bash"
    if "PYTHONPATH" not in os.environ: # Check if the environment variable is set
        if os.path.exists(ros_setup_file):
            # Source the setup file and capture environment variables
            command = f"bash -c 'source {ros_setup_file} && env'"
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, executable="/bin/bash")
            output, _ = proc.communicate()

            # Parse the environment variables
            env_vars = {}
            for line in output.decode("utf-8").splitlines():
                key, _, value = line.partition("=")
                env_vars[key] = value.strip()

            # Restart the script with the updated environment
            os.execvpe(sys.executable, [sys.executable] + sys.argv, {**os.environ, **env_vars})
        else:
            print(f"Error: ROS 2 setup file not found at {ros_setup_file}")
            sys.exit(1)