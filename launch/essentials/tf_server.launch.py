from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Advertise services for looking up transformations."""
    tf_server = Node(package="tf_server", executable="tf_server", name="tf_server", output="log")

    return LaunchDescription([tf_server])
