from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Run the main MoveIt executable without trajectory execution.

    We do not have controllers configured by default.
    """
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("amigo_moveit_config"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "allow_trajectory_execution": "true",
            "fake_execution": "false",
            "info": "false",
            "debug": "false",
        }.items(),
    )

    return LaunchDescription([move_group])
