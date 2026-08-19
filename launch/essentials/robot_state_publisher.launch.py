from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Publish the tf tree of the robot based on its URDF and joint states."""
    bringup_parameters = PathJoinSubstitution([EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters"])

    # Robot state publisher
    # TODO(ros2): ROS 2 has no global parameter server, so robot_state_publisher.yaml has to provide
    #             robot_description as a node parameter of this node.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            ParameterFile(
                PathJoinSubstitution([bringup_parameters, "essentials", "robot_state_publisher.yaml"]),
                allow_substs=True,
            )
        ],
    )

    return LaunchDescription([robot_state_publisher])
