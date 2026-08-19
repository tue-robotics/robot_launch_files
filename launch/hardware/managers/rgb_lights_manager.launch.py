from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Drive the RGB lights of the robot."""
    rgb_lights_manager = Node(
        package="rgb_lights_manager",
        executable="rgb_lights_manager",
        name="rgb_lights_manager",
        output="screen",
        remappings=[("/diagnostics", "diagnostics")],
    )

    return LaunchDescription([rgb_lights_manager])
