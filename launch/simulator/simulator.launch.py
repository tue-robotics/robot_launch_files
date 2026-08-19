from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the fast simulator and spawn the world, the objects and the robot in it."""
    robot_name = DeclareLaunchArgument("robot_name", default_value="amigo")
    launch_prefix = DeclareLaunchArgument(
        "launch_prefix", default_value="", description="Prefix the simulator is started with, e.g. gdb -ex run --args"
    )
    robot_env = DeclareLaunchArgument(
        "env", default_value=EnvironmentVariable("ROBOT_ENV", default_value="robotics_testlabs")
    )
    robot_init_x = DeclareLaunchArgument(
        "robot_init_x", default_value=EnvironmentVariable("ROBOT_INIT_X", default_value="0")
    )
    robot_init_y = DeclareLaunchArgument(
        "robot_init_y", default_value=EnvironmentVariable("ROBOT_INIT_Y", default_value="0")
    )
    robot_init_phi = DeclareLaunchArgument(
        "robot_init_phi", default_value=EnvironmentVariable("ROBOT_INIT_PHI", default_value="0")
    )

    environment = LaunchConfiguration("env")
    robot = LaunchConfiguration("robot_name")

    # start simulator
    fast_simulator = Node(
        package="fast_simulator",
        executable="simulator",
        name="fast_simulator",
        output="log",
        prefix=LaunchConfiguration("launch_prefix"),
    )

    # spawn world
    spawn_world = Node(
        package="fast_simulator",
        executable="spawn",
        name="spawn_world",
        output="log",
        arguments=["-i", environment, "-m", environment],
    )

    # spawn objects
    spawn_objects = Node(
        package="fast_simulator_data",
        executable="spawn_fast_simulator_objects.py",
        name="fast_simulator_object_spawner",
        output="log",
        arguments=[PathJoinSubstitution(["worlds", environment, "objects.yaml"])],
    )

    # spawn robot
    spawn_robot = Node(
        package="fast_simulator",
        executable="spawn",
        name="spawn_robot",
        output="log",
        arguments=[
            "-i",
            robot,
            "-m",
            robot,
            "-x",
            LaunchConfiguration("robot_init_x"),
            "-y",
            LaunchConfiguration("robot_init_y"),
            "-Z",
            LaunchConfiguration("robot_init_phi"),
        ],
    )

    ssl = Node(
        package="robot_launch_files",
        executable="ssl_dummy",
        name="ssl",
        parameters=[{"frame_id": [robot, TextSubstitution(text="/matrix")]}],
    )

    return LaunchDescription(
        [
            robot_name,
            launch_prefix,
            robot_env,
            robot_init_x,
            robot_init_y,
            robot_init_phi,
            fast_simulator,
            spawn_world,
            spawn_objects,
            spawn_robot,
            ssl,
        ]
    )
