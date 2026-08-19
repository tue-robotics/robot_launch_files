from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Build a map while driving using gmapping."""
    laser = DeclareLaunchArgument("laser", description="Namespace of the laser to map with")

    # TODO(ros2): gmapping has no ROS 2 release; slam_toolbox is the usual replacement
    gmapping_node = Node(
        package="gmapping",
        executable="slam_gmapping",
        name="gmapping_node",
        output="log",
        respawn=True,
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "localization", "gmapping.yaml"]
                ),
                allow_substs=True,
            )
        ],
        remappings=[
            ("scan", [LaunchConfiguration("laser"), TextSubstitution(text="/scan_gmapping")]),
            ("map", "gmapping/map"),
        ],
    )

    scan_gmapping = Node(
        package="robot_launch_files",
        executable="scan_gmapping.py",
        name="scan_gmapping",
        respawn=True,
        remappings=[
            ("scan", [LaunchConfiguration("laser"), TextSubstitution(text="/scan")]),
            ("scan_gmapping", [LaunchConfiguration("laser"), TextSubstitution(text="/scan_gmapping")]),
        ],
    )

    return LaunchDescription([laser, gmapping_node, scan_gmapping])
