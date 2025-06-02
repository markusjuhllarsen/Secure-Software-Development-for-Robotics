# Secure-Software-Development-for-Robotics
Using Ubuntu 24.04 Noble

Installing ROS 2 Jazzy Jalisco:  
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Installing Turtlebot simulation:  
https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html  

To build the interface and irobot_create_nodes packages (in outer directory of repo):  
source /opt/ros/jazzy/setup.bash  
colcon build 

To run the nodes:  
In one terminal (security node):   
source /opt/ros/jazzy/setup.bash  
source install/setup.bash  
python3 main_security.py

In another terminal (controller node):  
source /opt/ros/jazzy/setup.bash  
source install/setup.bash  
python3 main.py 

In a third terminal (turtlebot4 sim)  
source /opt/ros/jazzy/setup.bash  
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
