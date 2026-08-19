from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the torso server."""
    torso_server = Node(
        package="tue_manipulation",
        executable="torso_server",
        name="torso_server",
        output="screen",
        respawn=False,
        remappings=[
            ("torso_server/references", "torso/references"),
            ("torso_server/measurements", "torso/measurements"),
        ],
    )

    return LaunchDescription([torso_server])
