from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start the PicoVoice Rhino intent recognition driver.

    The access key must be stored in ~/MEGA/credentials/picovoice/access_key_<robot_name>.yaml.
    The file is loaded as a ROS 2 parameter file, so its content must be::

        /**:
          ros__parameters:
            access_key: "<ACCESS_KEY>"
    """
    robot_name = DeclareLaunchArgument("robot_name", description="Name of the robot")
    model_url = DeclareLaunchArgument(
        "model_url", default_value="package://picovoice_driver/extern/rhino/lib/common/rhino_params.pv"
    )
    contexts_directory_url = DeclareLaunchArgument(
        "contexts_directory_url",
        default_value=PathJoinSubstitution(
            [TextSubstitution(text="file://"), EnvironmentVariable("HOME"), "data", "picovoice", "contexts"]
        ),
    )
    credentials = PathJoinSubstitution([EnvironmentVariable("HOME"), "MEGA", "credentials", "picovoice"])
    picovoice_robot_config = DeclareLaunchArgument(
        "picovoice_robot_config",
        default_value=PathJoinSubstitution(
            [credentials, ["access_key_", LaunchConfiguration("robot_name"), TextSubstitution(text=".yaml")]]
        ),
    )
    picovoice_test_config = DeclareLaunchArgument(
        "picovoice_test_config", default_value=PathJoinSubstitution([credentials, "access_key_test.yaml"])
    )

    access_key_config = IfElseSubstitution(
        EnvironmentVariable("ROBOT_REAL", default_value="false"),
        if_value=LaunchConfiguration("picovoice_robot_config"),
        else_value=LaunchConfiguration("picovoice_test_config"),
    )

    # PicoVoice Driver Rhino
    picovoice_driver_rhino = Node(
        package="picovoice_driver",
        executable="picovoice_driver_rhino",
        name="picovoice_driver_rhino",
        output="log",
        respawn=True,
        parameters=[
            {
                "model_url": LaunchConfiguration("model_url"),
                "contexts_directory_url": LaunchConfiguration("contexts_directory_url"),
            },
            ParameterFile(access_key_config, allow_substs=True),
        ],
    )

    return LaunchDescription(
        [
            robot_name,
            model_url,
            contexts_directory_url,
            picovoice_robot_config,
            picovoice_test_config,
            picovoice_driver_rhino,
        ]
    )
