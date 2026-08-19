from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Exchange triggers with the other robot."""
    multirobot_parameters = PathJoinSubstitution(
        [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "multirobot"]
    )

    trigger = GroupAction(
        [
            Node(
                package="multirobot_communication",
                executable="trigger_client.py",
                name="trigger_client",
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution([multirobot_parameters, "trigger_client.yaml"]), allow_substs=True
                    )
                ],
            ),
            Node(
                package="multirobot_communication",
                executable="trigger_server.py",
                name="trigger_server",
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution([multirobot_parameters, "trigger_server.yaml"]), allow_substs=True
                    )
                ],
            ),
        ],
        condition=IfCondition(EnvironmentVariable("ROBOT_REAL", default_value="false")),
    )

    return LaunchDescription([trigger])
