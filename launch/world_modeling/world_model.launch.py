from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the world model and configure it."""
    robot_env = DeclareLaunchArgument(
        "robot_env", default_value=EnvironmentVariable("ROBOT_ENV", default_value="robotics_testlabs")
    )
    config = DeclareLaunchArgument("config", default_value="world_modeling/world_model.yaml")

    # World model
    ed = Node(package="ed", executable="ed", name="ed", output="log")

    # Configure world model
    ed_configure = Node(
        package="ed",
        executable="configure",
        name="ed_configure",
        output="log",
        arguments=[
            PathJoinSubstitution(
                [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", LaunchConfiguration("config")]
            )
        ],
    )

    return LaunchDescription([robot_env, config, ed, ed_configure])
