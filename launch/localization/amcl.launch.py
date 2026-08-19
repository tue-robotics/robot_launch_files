from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Localize the robot on a static map using AMCL."""
    laser = DeclareLaunchArgument("laser", description="Namespace of the laser to localize with")
    map_file = DeclareLaunchArgument("map", description="Map file to localize on")
    robot_init_x = DeclareLaunchArgument(
        "robot_init_x", default_value=EnvironmentVariable("ROBOT_INIT_X", default_value="0")
    )
    robot_init_y = DeclareLaunchArgument(
        "robot_init_y", default_value=EnvironmentVariable("ROBOT_INIT_Y", default_value="0")
    )
    robot_init_phi = DeclareLaunchArgument(
        "robot_init_phi", default_value=EnvironmentVariable("ROBOT_INIT_PHI", default_value="0")
    )

    # MAP SERVER
    # TODO(ros2): replace by nav2_map_server, which takes the map as a yaml_filename parameter
    loc_map_server = Node(
        package="map_server",
        executable="map_server",
        name="loc_map_server",
        output="log",
        arguments=[LaunchConfiguration("map")],
        remappings=[("map", "loc_map")],
    )

    # AMCL
    # TODO(ros2): replace by nav2_amcl
    amcl = Node(
        package="amcl",
        executable="amcl",
        name="amcl",
        output="log",
        remappings=[
            ("scan", [LaunchConfiguration("laser"), TextSubstitution(text="/scan")]),
            ("map", "loc_map"),
            ("/diagnostics", "diagnostics"),
        ],
        parameters=[
            {
                "initial_pose_x": ParameterValue(LaunchConfiguration("robot_init_x"), value_type=float),
                "initial_pose_y": ParameterValue(LaunchConfiguration("robot_init_y"), value_type=float),
                "initial_pose_a": ParameterValue(LaunchConfiguration("robot_init_phi"), value_type=float),
            },
            ParameterFile(
                PathJoinSubstitution(
                    [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "localization", "amcl.yaml"]
                ),
                allow_substs=True,
            ),
        ],
    )

    return LaunchDescription([laser, map_file, robot_init_x, robot_init_y, robot_init_phi, loc_map_server, amcl])
