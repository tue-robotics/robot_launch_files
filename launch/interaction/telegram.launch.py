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
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Bridge Telegram messages and images to and from ROS.

    The token must be stored in ~/MEGA/credentials/telegram/token_<robot_name>.yaml.
    The content of that file must be::

        token: "API_TOKEN"
    """
    robot_name = DeclareLaunchArgument("robot_name", description="Name of the robot")
    caption_as_frame_id = DeclareLaunchArgument("caption_as_frame_id", default_value="false")

    credentials = PathJoinSubstitution([EnvironmentVariable("HOME"), "MEGA", "credentials", "telegram"])
    telegram_robot_config = DeclareLaunchArgument(
        "telegram_robot_config",
        default_value=PathJoinSubstitution(
            [credentials, ["token_", LaunchConfiguration("robot_name"), TextSubstitution(text=".yaml")]]
        ),
    )
    telegram_test_config = DeclareLaunchArgument(
        "telegram_test_config", default_value=PathJoinSubstitution([credentials, "token_test.yaml"])
    )

    token_config = IfElseSubstitution(
        EnvironmentVariable("ROBOT_REAL", default_value="false"),
        if_value=LaunchConfiguration("telegram_robot_config"),
        else_value=LaunchConfiguration("telegram_test_config"),
    )

    # Telegram ROS Bridge
    telegram_ros_bridge = Node(
        package="telegram_ros",
        executable="telegram_ros_bridge",
        name="telegram_ros_bridge",
        output="screen",
        respawn=True,
        parameters=[
            ParameterFile(token_config, allow_substs=True),
            {"caption_as_frame_id": ParameterValue(LaunchConfiguration("caption_as_frame_id"), value_type=bool)},
        ],
    )

    return LaunchDescription(
        [robot_name, caption_as_frame_id, telegram_robot_config, telegram_test_config, telegram_ros_bridge]
    )
