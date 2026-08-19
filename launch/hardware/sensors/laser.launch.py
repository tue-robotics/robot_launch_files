from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start a laser range finder and its filter chain in its own namespace."""
    name = DeclareLaunchArgument("name", description="Namespace and parameter file name of the laser")

    sensor_parameters = PathJoinSubstitution(
        [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "hardware", "sensors"]
    )

    # Push down all topics into the laser namespace
    laser = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("name")),
            # TODO(ros2): urg_node is not released for ROS 2 in this workspace yet
            Node(
                package="urg_node",
                executable="urg_node",
                name="urg_node",
                output="screen",
                remappings=[("scan", "scan_raw"), ("/diagnostics", "diagnostics")],
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution(
                            [
                                sensor_parameters,
                                [LaunchConfiguration("name"), TextSubstitution(text="_parameters.yaml")],
                            ]
                        ),
                        allow_substs=True,
                    )
                ],
            ),
            # TODO(ros2): laser_filters is not released for ROS 2 in this workspace yet
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="laser_scan_to_scan_filter_chain",
                remappings=[("scan", "scan_raw"), ("scan_filtered", "scan")],
                parameters=[
                    ParameterFile(
                        PathJoinSubstitution(
                            [
                                sensor_parameters,
                                [LaunchConfiguration("name"), TextSubstitution(text="_filters_parameters.yaml")],
                            ]
                        ),
                        allow_substs=True,
                    )
                ],
            ),
        ]
    )

    return LaunchDescription([name, laser])
