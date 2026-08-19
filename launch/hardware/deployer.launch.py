from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AnonName, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the Orocos deployer that runs the hardware components.

    TODO(ros2): rtt_ros/Orocos RTT has no ROS 2 release; this file describes the intended structure only.
    """
    name = DeclareLaunchArgument("NAME", default_value=AnonName("deployer"), description="Node name for the deployer")
    log_level = DeclareLaunchArgument(
        "LOG_LEVEL",
        default_value="info",
        description="Orocos log level (never, fatal, critical, error, warning, info, debug, realtime)",
    )
    # TODO(ros2): ROS 2 launch does not word-split arguments, so DEPLOYER_ARGS is passed as a single argument
    deployer_args = DeclareLaunchArgument(
        "DEPLOYER_ARGS", default_value="", description="Additional Orocos deployer args"
    )
    orocos_target = DeclareLaunchArgument(
        "OROCOS_TARGET",
        default_value=EnvironmentVariable("OROCOS_TARGET", default_value=""),
        description="Orocos target (gnulinux, xenomai, etc)",
    )
    rtt_component_path = DeclareLaunchArgument(
        "RTT_COMPONENT_PATH",
        default_value=EnvironmentVariable("RTT_COMPONENT_PATH"),
        description="Path to look for dynamically-loaded plugins and components (something like /lib/orocos)",
    )
    debug = DeclareLaunchArgument(
        "DEBUG", default_value="false", description="Run in GDB (don't forget to build in Debug mode)"
    )

    additional_env = {
        "OROCOS_TARGET": LaunchConfiguration("OROCOS_TARGET"),
        "RTT_COMPONENT_PATH": LaunchConfiguration("RTT_COMPONENT_PATH"),
    }

    # Launch deployer
    deployer = Node(
        package="rtt_ros",
        executable="deployer",
        name=LaunchConfiguration("NAME"),
        arguments=["-l", LaunchConfiguration("LOG_LEVEL"), LaunchConfiguration("DEPLOYER_ARGS"), "--"],
        output="screen",
        additional_env=additional_env,
        condition=IfCondition(LaunchConfiguration("DEBUG")),
    )

    rttscript = Node(
        package="rtt_ros",
        executable="rttscript",
        name="hardware",
        arguments=[
            "-l",
            LaunchConfiguration("LOG_LEVEL"),
            LaunchConfiguration("DEPLOYER_ARGS"),
            "-s",
            PathJoinSubstitution(
                [FindPackageShare("robot_launch_files"), "launch", "hardware", "wait_for_interrupt.ops"]
            ),
            "--",
        ],
        output="log",
        respawn=True,
        additional_env=additional_env,
        condition=UnlessCondition(LaunchConfiguration("DEBUG")),
    )

    # Launch ROSlogger
    hardwarelog = Node(package="rtt_control_components", executable="orocoslog2roslog.py", name="hardwarelog")

    return LaunchDescription(
        [name, log_level, deployer_args, orocos_target, rtt_component_path, debug, deployer, rttscript, hardwarelog]
    )
