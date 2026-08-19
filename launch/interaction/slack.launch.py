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
    """Bridge Slack messages and images to and from ROS.

    The token must be stored in ~/MEGA/credentials/slack/token_<robot_name>.yaml.
    The file is loaded as a ROS 2 parameter file, so its content must be::

        /**:
          ros__parameters:
            token: "API_TOKEN"
    """
    robot_name = DeclareLaunchArgument("robot_name", description="Name of the robot")

    credentials = PathJoinSubstitution([EnvironmentVariable("HOME"), "MEGA", "credentials", "slack"])
    token_config = IfElseSubstitution(
        EnvironmentVariable("ROBOT_REAL", default_value="false"),
        if_value=PathJoinSubstitution(
            [credentials, ["token_", LaunchConfiguration("robot_name"), TextSubstitution(text=".yaml")]]
        ),
        else_value=PathJoinSubstitution([credentials, "token_test.yaml"]),
    )

    # Slack ROS Bridge
    slack_ros_bridge = Node(
        package="slack_ros",
        executable="slack_ros_bridge",
        name="slack_ros_bridge",
        output="screen",
        respawn=True,
        parameters=[ParameterFile(token_config, allow_substs=True)],
        remappings=[
            ("message_to_ros", "message_from_user"),
            ("message_from_ros", "message_to_user"),
            ("image_to_ros", "photo_from_user"),
            ("image_from_ros", "photo_to_user"),
        ],
    )

    return LaunchDescription([robot_name, slack_ros_bridge])
