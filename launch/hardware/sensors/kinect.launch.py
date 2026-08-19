from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start the driver of an RGBD camera in its own namespace."""
    name = DeclareLaunchArgument("name", description="Namespace and parameter file name of the camera")

    camera = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("name")),
            Node(
                package="kinect_driver",
                executable="kinect_driver",
                name="driver",
                respawn=True,
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution(
                            [
                                EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                                "parameters",
                                "hardware",
                                "sensors",
                                [LaunchConfiguration("name"), TextSubstitution(text=".yaml")],
                            ]
                        ),
                        allow_substs=True,
                    )
                ],
            ),
        ]
    )

    return LaunchDescription([name, camera])
