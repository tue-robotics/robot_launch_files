from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the touchscreen trigger node."""
    touchscreen = Node(
        package="picaso_4d_systems",
        executable="picaso_4d_systems_trigger_node",
        name="touchscreen",
        output="screen",
        respawn=False,
        parameters=[{"serialport": "/dev/ttyUSB0"}],
    )

    return LaunchDescription([touchscreen])
