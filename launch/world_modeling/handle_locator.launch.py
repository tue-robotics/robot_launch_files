from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Locate handles of doors and drawers."""
    # Handle recognition
    handle_locator = GroupAction(
        [
            PushRosNamespace("handle_locator"),
            Node(
                package="handle_locator", executable="locate_handle_action", name="locate_handle_action", output="log"
            ),
        ]
    )

    return LaunchDescription([handle_locator])
