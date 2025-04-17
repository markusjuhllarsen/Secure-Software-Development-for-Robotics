from setuptools import setup, find_packages

setup(
    name="secure_turtlebot_controller",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        'rclpy',
    ],
    author="[Your Name]",
    author_email="[Your Email]",
    description="A secure interface for controlling Turtlebot4 robots",
    keywords="robotics, security, turtlebot4, ROS2",
    entry_points={
        'console_scripts': [
            'secure_turtlebot_controller=main:main',
        ],
    },
)