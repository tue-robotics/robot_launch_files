from launch import LaunchDescription
from launch.substitutions import AnonName, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start rviz with the configuration of the robot."""
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name=AnonName("rviz"),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "tools", "rviz_config.rviz"]
            ),
        ],
    )

    return LaunchDescription([rviz])
