from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Aggregate the diagnostics of all nodes and monitor whether they are still alive."""
    bringup_parameters = PathJoinSubstitution([EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters"])

    # Collects all the diagnostics messages
    # TODO(ros2): diagnostic_aggregator is not released for ROS 2 in this workspace yet
    diagnostics_aggregator = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostics_aggegrator",
        output="screen",
        respawn=False,
        arguments=["CPP"],
        parameters=[
            ParameterFile(
                PathJoinSubstitution([bringup_parameters, "diagnostics", "aggegrator.yaml"]), allow_substs=True
            )
        ],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/diagnostics_agg", "diagnostics_agg"),
            ("/diagnostics_toplevel_state", "diagnostics_toplevel_state"),
        ],
    )

    # Checks if all nodes are still running
    node_alive_server = Node(
        package="node_alive",
        executable="node_alive_server",
        name="node_alive_server",
        output="log",
        parameters=[
            ParameterFile(
                PathJoinSubstitution([bringup_parameters, "diagnostics", "node_alive_neglect_nodes.yaml"]),
                allow_substs=True,
            )
        ],
    )

    # TODO(ros2): Start challenge logger; the challenge_logger needs cpu_monitor information

    return LaunchDescription([diagnostics_aggregator, node_alive_server])
