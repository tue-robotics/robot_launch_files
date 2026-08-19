from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start the dynamixel controller manager and spawn the requested joint controllers."""
    controller_args = DeclareLaunchArgument(
        "dynamixel_controller_args", description="Names of the joint controllers to spawn"
    )

    # ROS 2 has no global parameter server; the controller configuration is loaded onto the manager node itself
    controller_config = ParameterFile(
        PathJoinSubstitution(
            [
                EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                "parameters",
                "hardware",
                "actuators",
                "dynamixel_joint_controllers.yaml",
            ]
        ),
        allow_substs=True,
    )

    manager = Node(
        package="dynamixel_controllers",
        executable="controller_manager.py",
        name="dynamixel_manager",
        parameters=[controller_config],
        on_exit=Shutdown(),
    )

    # start specified joint controllers
    spawner = Node(
        package="dynamixel_controllers",
        executable="controller_spawner.py",
        name="dynamixel_controller_spawner",
        arguments=[LaunchConfiguration("dynamixel_controller_args")],
        output="screen",
    )

    return LaunchDescription([controller_args, manager, spawner])
