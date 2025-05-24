# Secure-Software-Development-for-Robotics

To build the interface and irobot_create_nodes packages(in outer directory of repo):  
source /opt/ros/jazzy/setup.bash  
colcon build 

To run the nodes:  
In one terminal:   
source /opt/ros/jazzy/setup.bash  
source install/setup.bash  
main_security.py (security node)

In another terminal:  
source /opt/ros/jazzy/setup.bash  
source install/setup.bash  
main.py (controller node)



Run turtlebot4 sim
1. source /opt/ros/jazzy/setup.bash
2. chmod +x main.py
3. python3 main.py


# Secure-Software-Development-for-Robotics

Using Ubuntu 24.04 Noble (also through WSL2)

# ROS2 Setup and Installation
We require both the simulation and control hosts to have ROS2 installed (Jazzy Jalisco version).

This can be done using the deb packages (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):

sudo apt install software-properties-common  
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y  
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Optional for development tools:  
sudo apt update && sudo apt install ros-dev-tools

sudo apt update  
sudo apt upgrade  
sudo apt install ros-jazzy-desktop  

To setup the environment we then need to source the setup file:  
source /opt/ros/jazzy/setup.bash

This needs to be done for every terminal every time.

# Turtlebot4 Setup and Installation
For the simulation host we use Gazebo to simulate the Turtlebot 4.  We use Gazebo Harmonic to do this (https://gazebosim.org/docs/harmonic/install_ubuntu/).

sudo apt-get update  
sudo apt-get install curl lsb-release gnupg  

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg  
echo "deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable \$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null  
sudo apt-get update  
sudo apt-get install gz-harmonic  

To install the Turtlebot4 simulation:  
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes
