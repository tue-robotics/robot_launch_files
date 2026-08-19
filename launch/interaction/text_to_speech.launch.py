from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start text to speech and the audio player, festival in simulation and Philips on the real robot."""
    robot_real = EnvironmentVariable("ROBOT_REAL", default_value="false")

    # In ROS 1 these were written to the parameter server under the /text_to_speech namespace, which is the
    # private namespace of the text_to_speech node. ROS 2 has no parameter server, so they are node parameters.
    simulation_parameters = [{"tts_module": "festival"}]
    real_parameters = [
        # TODO(ros2): ROS 2 has no textfile parameter; philips_text_to_speech has to read the key file itself
        {"key_file": PathJoinSubstitution([FindPackageShare("philips_text_to_speech"), "key"])},
        ParameterFile(
            PathJoinSubstitution(
                [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "interaction", "text_to_speech.yaml"]
            ),
            allow_substs=True,
        ),
    ]

    # Start TTS node
    text_to_speech = {
        "package": "text_to_speech",
        "executable": "text_to_speech_node.py",
        "name": "text_to_speech",
        "output": "log",
        "respawn": False,
        "remappings": [("play", "audio_player/play")],
    }
    simulation_tts = Node(**text_to_speech, parameters=simulation_parameters, condition=UnlessCondition(robot_real))
    real_tts = Node(**text_to_speech, parameters=real_parameters, condition=IfCondition(robot_real))

    # Start play node
    audio_player = Node(
        package="text_to_speech", executable="player.py", name="audio_player", output="log", respawn=False
    )

    return LaunchDescription([simulation_tts, real_tts, audio_player])
