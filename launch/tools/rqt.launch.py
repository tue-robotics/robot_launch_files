from launch import LaunchDescription
from launch.substitutions import AnonName, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start rqt with the perspective and rviz configuration of the robot."""
    tools_parameters = PathJoinSubstitution([EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "tools"])

    rqt = Node(
        package="robot_launch_files",
        executable="rqt.bash",
        name=AnonName("rqt"),
        arguments=[
            PathJoinSubstitution([tools_parameters, "rqt.perspective"]),
            PathJoinSubstitution([tools_parameters, "rviz_config.rviz"]),
        ],
    )

    return LaunchDescription([rqt])
