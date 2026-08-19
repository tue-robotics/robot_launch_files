from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Plan and follow paths for the base.

    TODO(ros2): ROS 1 loaded the costmap files into a sub-namespace of the node. ROS 2 has no per-node parameter
                namespaces, so local_costmap_gmapping.yaml and global_costmap_gmapping.yaml have to nest their
                content under a local_costmap resp. global_costmap key inside ros__parameters.
    """
    robot_real = EnvironmentVariable("ROBOT_REAL", default_value="false")
    use_navscan = DeclareLaunchArgument("use_navscan", default_value=robot_real)

    navigation_parameters = PathJoinSubstitution(
        [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "navigation"]
    )

    # Local planner that sends cmd_vels to the robot_base
    local_planner = {
        "package": "cb_base_navigation",
        "executable": "local_planner",
        "name": "local_planner",
        "output": "log",
        "respawn": True,
        # Topic remapping
        "remappings": [("odom", "base/measurements"), ("cmd_vel", "base/references")],
    }
    local_planner_parameters = [
        # Local Planner Settings
        ParameterFile(PathJoinSubstitution([navigation_parameters, "local_planner.yaml"]), allow_substs=True),
        # Local Costmap Settings
        ParameterFile(PathJoinSubstitution([navigation_parameters, "local_costmap_gmapping.yaml"]), allow_substs=True),
    ]
    simulation_local_costmap = {"local_costmap.recent_obstacles.observation_sources": "base_laser"}

    real_local_planner = Node(**local_planner, parameters=local_planner_parameters, condition=IfCondition(robot_real))
    simulation_local_planner = Node(
        **local_planner,
        parameters=[*local_planner_parameters, simulation_local_costmap],
        condition=UnlessCondition(robot_real),
    )

    # Global planner for path calculation from origin pose to goal constraint region
    global_planner = {
        "package": "cb_base_navigation",
        "executable": "global_planner",
        "name": "global_planner",
        "output": "log",
        "respawn": True,
    }
    global_planner_parameters = [
        # Global Planner Settings
        ParameterFile(PathJoinSubstitution([navigation_parameters, "global_planner.yaml"]), allow_substs=True),
        # Global Costmap Settings
        ParameterFile(PathJoinSubstitution([navigation_parameters, "global_costmap_gmapping.yaml"]), allow_substs=True),
    ]
    simulation_global_costmap = {"global_costmap.recent_obstacles.observation_sources": "base_laser"}

    real_global_planner = Node(
        **global_planner, parameters=global_planner_parameters, condition=IfCondition(robot_real)
    )
    simulation_global_planner = Node(
        **global_planner,
        parameters=[*global_planner_parameters, simulation_global_costmap],
        condition=UnlessCondition(robot_real),
    )

    # RGBD to navscan for obstacle avoidance
    depthimage_to_navscan = Node(
        package="depthimage_to_navscan_rgbd",
        executable="depthimage_to_navscan_rgbd",
        name="depthimage_to_navscan",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_navscan")),
        # Navscan Settings
        parameters=[ParameterFile(PathJoinSubstitution([navigation_parameters, "navscan.yaml"]), allow_substs=True)],
    )

    return LaunchDescription(
        [
            use_navscan,
            real_local_planner,
            simulation_local_planner,
            real_global_planner,
            simulation_global_planner,
            depthimage_to_navscan,
        ]
    )
