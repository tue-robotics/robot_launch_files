from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the configured localization implementation."""
    localization_type = DeclareLaunchArgument("type", default_value="amcl", description="Localization implementation")
    laser = DeclareLaunchArgument("laser", description="Namespace of the laser to localize with")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_launch_files"),
                    "launch",
                    "localization",
                    [LaunchConfiguration("type"), TextSubstitution(text=".launch.py")],
                ]
            )
        ),
        launch_arguments={"laser": LaunchConfiguration("laser")}.items(),
    )

    return LaunchDescription([localization_type, laser, localization])
