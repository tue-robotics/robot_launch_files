from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start the head reference server and its cmd_vel client."""
    manipulation_parameters = PathJoinSubstitution(
        [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "manipulation"]
    )

    # HEAD REF SERVER
    head_ref = Node(
        package="head_ref",
        executable="head_ref",
        name="head_ref",
        output="log",
        parameters=[
            ParameterFile(PathJoinSubstitution([manipulation_parameters, "head_server.yaml"]), allow_substs=True)
        ],
        remappings=[
            ("/neck/references", "neck/references"),
            ("/neck/measurements", "neck/measurements"),
            ("pan_controller/command", "neck_pan/command"),
            ("tilt_controller/command", "neck_tilt/command"),
        ],
    )

    # head_ref_client_look_around = Node(
    #     package="head_ref",
    #     executable="look_around_client",
    #     name="head_ref_client_look_around",
    #     output="log",
    #     parameters=[
    #         ParameterFile(
    #             PathJoinSubstitution([manipulation_parameters, "head_look_around_client.yaml"]), allow_substs=True
    #         )
    #     ],
    #     remappings=[("/cmd_vel", "base/references")],
    # )

    head_ref_client_cmd_vel = Node(
        package="head_ref",
        executable="cmd_vel_client",
        name="head_ref_client_cmd_vel",
        output="log",
        parameters=[
            ParameterFile(
                PathJoinSubstitution([manipulation_parameters, "head_cmd_vel_client.yaml"]), allow_substs=True
            )
        ],
        remappings=[("/cmd_vel", "base/references")],
    )

    return LaunchDescription([head_ref, head_ref_client_cmd_vel])
