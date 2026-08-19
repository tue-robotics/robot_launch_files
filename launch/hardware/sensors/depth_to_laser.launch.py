from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Project the depth image of the head camera onto a laser scan."""
    robot_name = DeclareLaunchArgument("robot_name", description="Name of the robot")
    camera_info_depth = DeclareLaunchArgument(
        "camera_info_depth", default_value="head_rgbd_sensor/depth_registered/camera_info"
    )
    image_depth = DeclareLaunchArgument("image_depth", default_value="head_rgbd_sensor/depth_registered/image")
    scan_projected = DeclareLaunchArgument("scan_projected", default_value="head_rgbd_sensor/scan")

    # ROS 2 static_transform_publisher takes named arguments instead of a single positional transform string
    camera_laser_x = DeclareLaunchArgument("camera_laser_x", default_value="0")
    camera_laser_y = DeclareLaunchArgument("camera_laser_y", default_value="0")
    camera_laser_z = DeclareLaunchArgument("camera_laser_z", default_value="0")
    camera_laser_yaw = DeclareLaunchArgument("camera_laser_yaw", default_value="0")
    camera_laser_pitch = DeclareLaunchArgument("camera_laser_pitch", default_value="-1.57")
    camera_laser_roll = DeclareLaunchArgument("camera_laser_roll", default_value="1.57")

    camera_frame = DeclareLaunchArgument("camera_frame", default_value="head_rgbd_sensor_rgb_frame")
    laser_frame = DeclareLaunchArgument("laser_frame", default_value="head_rgbd_sensor_laser_frame")

    # TODO(ros2): depthimage_to_laserscan is not released for ROS 2 in this workspace yet
    depthimage_to_laserscan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan",
        name="depthimage_to_laserscan",
        output="screen",
        remappings=[
            ("image", LaunchConfiguration("image_depth")),
            ("camera_info", LaunchConfiguration("camera_info_depth")),
            ("scan", LaunchConfiguration("scan_projected")),
        ],
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [
                        EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                        "parameters",
                        "hardware",
                        "sensors",
                        "head_camera_laser_projection.yaml",
                    ]
                ),
                allow_substs=True,
            )
        ],
    )

    # transform between camera frame to laser frame
    camera_laser_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="head_camera_laser_broadcaster",
        arguments=[
            "--x",
            LaunchConfiguration("camera_laser_x"),
            "--y",
            LaunchConfiguration("camera_laser_y"),
            "--z",
            LaunchConfiguration("camera_laser_z"),
            "--yaw",
            LaunchConfiguration("camera_laser_yaw"),
            "--pitch",
            LaunchConfiguration("camera_laser_pitch"),
            "--roll",
            LaunchConfiguration("camera_laser_roll"),
            "--frame-id",
            LaunchConfiguration("camera_frame"),
            "--child-frame-id",
            LaunchConfiguration("laser_frame"),
        ],
    )

    return LaunchDescription(
        [
            robot_name,
            camera_info_depth,
            image_depth,
            scan_projected,
            camera_laser_x,
            camera_laser_y,
            camera_laser_z,
            camera_laser_yaw,
            camera_laser_pitch,
            camera_laser_roll,
            camera_frame,
            laser_frame,
            depthimage_to_laserscan,
            camera_laser_broadcaster,
        ]
    )
