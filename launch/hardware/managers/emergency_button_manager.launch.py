from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Speak up when the emergency button is pressed."""
    emergency_speakup = Node(
        package="emergency_speakup", executable="emergency_speakup", name="emergency_speakup", respawn=True
    )

    return LaunchDescription([emergency_speakup])
