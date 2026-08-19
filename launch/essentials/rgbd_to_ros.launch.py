from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Convert an rgbd stream back to separate ROS image topics and expose a 3D point query service."""
    name = DeclareLaunchArgument("name", description="Namespace of the camera to convert")

    camera = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("name")),
            # rgbd to ros conversion
            Node(package="rgbd", executable="rgbd_to_ros", name="rgbd_to_ros", arguments=["rgbd"], respawn=True),
            # Expose interface to query 3d points
            Node(
                package="rgbd",
                executable="get_3d_point_from_image_roi_node",
                name="project_3d_point_from_image_roi",
                respawn=True,
            ),
        ]
    )

    return LaunchDescription([name, camera])
