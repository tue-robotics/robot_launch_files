from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start the gripper server and grasp precompute action for one arm."""
    name = DeclareLaunchArgument("name", description="Name of the arm")

    arm = LaunchConfiguration("name")

    # GRIPPER SERVER
    gripper_server = Node(
        package="tue_manipulation",
        executable="gripper_server",
        name=["gripper_server_", arm],
        output="log",
        respawn=False,
        remappings=[
            ("action", [arm, TextSubstitution(text="/gripper/action")]),
            ("references", [arm, TextSubstitution(text="/gripper/references")]),
            ("measurements", [arm, TextSubstitution(text="/gripper/measurements")]),
        ],
    )

    # GRASP PRECOMPUTE
    grasp_precompute = Node(
        package="tue_manipulation",
        executable="grasp_precompute_action",
        name=["grasp_precompute_", arm],
        output="log",
        remappings=[
            ("joint_trajectory_action", [arm, TextSubstitution(text="/joint_trajectory_action")]),
            ("joint_trajectory", [arm, TextSubstitution(text="/joint_trajectory")]),
            ("grasp_precompute", [arm, TextSubstitution(text="/grasp_precompute")]),
            ("ik_position_markers", [arm, TextSubstitution(text="/ik_position_markers")]),
            # Measurement topics
            ("joint_measurements", [arm, TextSubstitution(text="/measurements")]),
            ("spindle_measurement", "torso/measurements"),
        ],
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [
                        EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                        "parameters",
                        "manipulation",
                        arm,
                        "grasp_precompute.yaml",
                    ]
                ),
                allow_substs=True,
            )
        ],
    )

    return LaunchDescription([name, gripper_server, grasp_precompute])
