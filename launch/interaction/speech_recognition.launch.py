from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Start the speech recognition stack, either the real one or a dummy in simulation."""
    dragonfly = DeclareLaunchArgument("dragonfly", default_value="true")
    kaldi = DeclareLaunchArgument("kaldi", default_value="false")
    qr_decoder_image_topic = DeclareLaunchArgument("qr_decoder_image_topic", default_value="~/image")
    model_path = DeclareLaunchArgument(
        "model_path",
        default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "data", "speech_models", "model"]),
    )

    robot_real = EnvironmentVariable("ROBOT_REAL", default_value="false")

    # Always run a multicast server with a string topic answerer for amigo-hear
    multi_client = Node(package="hmi", executable="multi_client", name="hmi", output="log")

    hmi = GroupAction(
        [
            PushRosNamespace("hmi"),
            # QR-code / amigo-hear
            Node(package="hmi", executable="string_topic_answerer", name="string_topic_answerer", output="log"),
            Node(
                package="hmi",
                executable="qr_code_decoder",
                name="qr_code_decoder",
                output="log",
                remappings=[("~/image", LaunchConfiguration("qr_decoder_image_topic"))],
            ),
            # If we are on the real robot, launch the server, otherwise a dummy
            GroupAction(
                [
                    # bridge to windows
                    Node(
                        package="dragonfly_speech_recognition",
                        executable="hmi_server_dragonfly_client",
                        name="dragonfly_speech_recognition",
                        output="log",
                        condition=IfCondition(LaunchConfiguration("dragonfly")),
                        parameters=[
                            ParameterFile(
                                PathJoinSubstitution(
                                    [
                                        EnvironmentVariable("ROBOT_BRINGUP_PATH"),
                                        "parameters",
                                        "interaction",
                                        "speech_client.yaml",
                                    ]
                                ),
                                allow_substs=True,
                            )
                        ],
                    ),
                    # Kaldi
                    Node(
                        package="speech_recognition",
                        executable="hmi_kaldi_node",
                        name="kaldi_speech_recognition",
                        output="log",
                        condition=IfCondition(LaunchConfiguration("kaldi")),
                        parameters=[{"kaldi_model_path": LaunchConfiguration("model_path")}],
                    ),
                ],
                condition=IfCondition(robot_real),
            ),
            GroupAction(
                [
                    # speech dummy
                    Node(package="hmi", executable="random_answerer", name="random_answerer", output="log"),
                    Node(
                        package="hmi",
                        executable="dragonfly_restart_mock",
                        name="dragonfly_speech_recognition",
                        output="log",
                    ),
                ],
                condition=UnlessCondition(robot_real),
            ),
        ]
    )

    return LaunchDescription([dragonfly, kaldi, qr_decoder_image_topic, model_path, multi_client, hmi])
