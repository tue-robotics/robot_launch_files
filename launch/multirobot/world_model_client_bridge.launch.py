from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Receive the world model of the other robot."""
    other_robot = DeclareLaunchArgument("other_robot", description="Name of the other robot")

    bridge = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("other_robot")),
            GroupAction(
                [
                    Node(
                        package="multirobot_communication",
                        executable="ed_client_bridge.py",
                        name="ed",
                        parameters=[
                            ParameterFile(
                                PathJoinSubstitution(
                                    [
                                        EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                                        "parameters",
                                        "multirobot",
                                        "world_model_client_bridge.yaml",
                                    ]
                                ),
                                allow_substs=True,
                            )
                        ],
                    )
                ],
                condition=IfCondition(EnvironmentVariable("ROBOT_REAL", default_value="false")),
            ),
        ]
    )

    return LaunchDescription([other_robot, bridge])
