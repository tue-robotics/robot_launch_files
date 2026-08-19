from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Serve the vizbox operator interface."""
    image = DeclareLaunchArgument("image", default_value="top_kinect/rgb/image")
    robot_name = DeclareLaunchArgument("robot_name", description="Name of the robot")
    param_file = DeclareLaunchArgument("param_file", description="Parameter file describing the challenge")

    vizbox = Node(
        package="vizbox",
        executable="server.py",
        name="vizbox",
        output="log",
        # ROS 1 cwd="node" ran the node from its package directory
        cwd=FindPackageShare("vizbox"),
        # In ROS 1 this file was loaded onto the global parameter server; ROS 2 loads it onto the node
        parameters=[ParameterFile(LaunchConfiguration("param_file"), allow_substs=True)],
        remappings=[
            ("robot_text", "text_to_speech/output"),
            ("operator_text", "hmi/result/sentence"),
            ("image", "vizbox/image/throttled"),
        ],
    )

    extract_hmi_result_sentence = Node(
        package="topic_tools",
        executable="transform",
        name="extract_hmi_result_sentence",
        arguments=[
            ["/", LaunchConfiguration("robot_name"), TextSubstitution(text="/hmi/result")],
            "hmi/result/sentence",
            "std_msgs/String",
            "m.result.sentence",
        ],
    )

    throttle_vizbox_img = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_vizbox_img",
        arguments=["messages", LaunchConfiguration("image"), "5.0", "vizbox/image/throttled"],
    )

    return LaunchDescription([image, robot_name, param_file, vizbox, extract_hmi_result_sentence, throttle_vizbox_img])
