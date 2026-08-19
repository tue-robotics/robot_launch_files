from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Monitor the battery state of the robot."""
    battery_manager = Node(
        package="battery_manager",
        executable="battery_manager",
        name="battery_manager",
        output="screen",
        respawn=False,
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [
                        EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                        "parameters",
                        "hardware",
                        "managers",
                        "battery_parameters.yaml",
                    ]
                ),
                allow_substs=True,
            )
        ],
    )

    return LaunchDescription([battery_manager])
