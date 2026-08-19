from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Share the world model of this robot with the other robot."""
    bridge = GroupAction(
        [
            Node(
                package="multirobot_communication",
                executable="ed_server_bridge.py",
                name="sync_ed",
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution(
                            [
                                EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                                "parameters",
                                "multirobot",
                                "world_model_server_bridge.yaml",
                            ]
                        ),
                        allow_substs=True,
                    )
                ],
            )
        ],
        condition=IfCondition(EnvironmentVariable("ROBOT_REAL", default_value="false")),
    )

    return LaunchDescription([bridge])
