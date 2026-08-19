from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Recognize objects using tensorflow."""
    robot_env = DeclareLaunchArgument(
        "robot_env", default_value=EnvironmentVariable("ROBOT_ENV", default_value="robotics_testlabs")
    )
    robot_real = DeclareLaunchArgument(
        "robot_real", default_value=EnvironmentVariable("ROBOT_REAL", default_value="false")
    )

    models = ["~/MEGA/data/", LaunchConfiguration("robot_env"), TextSubstitution(text="/models")]
    tensorflow_graph = DeclareLaunchArgument(
        "tensorflow_graph",
        default_value=[*models, TextSubstitution(text="/image_recognition_tensorflow/output_graph.pb")],
    )
    tensorflow_labels = DeclareLaunchArgument(
        "tensorflow_labels",
        default_value=[*models, TextSubstitution(text="/image_recognition_tensorflow/output_labels.txt")],
    )

    # Object recognition
    object_recognition = GroupAction(
        [
            PushRosNamespace("object_recognition"),
            Node(
                package="image_recognition_tensorflow",
                executable="object_recognition_node",
                name="object_recognition",
                output="log",
                parameters=[
                    {
                        "graph_path": LaunchConfiguration("tensorflow_graph"),
                        "labels_path": LaunchConfiguration("tensorflow_labels"),
                        "save_images": ParameterValue(LaunchConfiguration("robot_real"), value_type=bool),
                        "save_images_folder": [
                            "~/MEGA/data/",
                            LaunchConfiguration("robot_env"),
                            TextSubstitution(text="/training_data"),
                        ],
                    }
                ],
            ),
        ]
    )

    return LaunchDescription([robot_env, robot_real, tensorflow_graph, tensorflow_labels, object_recognition])
