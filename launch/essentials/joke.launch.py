from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Tell a joke whenever the robot has been standing by for too long."""
    joke = Node(package="robot_launch_files", executable="joke.py", name="joke")

    return LaunchDescription([joke])
