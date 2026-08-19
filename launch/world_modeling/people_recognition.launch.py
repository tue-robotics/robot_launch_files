from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Recognize people, their faces, their age and gender and the colour of their shirt.

    pose_estimation should be launched manually.
    """
    robot_env = DeclareLaunchArgument(
        "robot_env", default_value=EnvironmentVariable("ROBOT_ENV", default_value="robotics_testlabs")
    )
    robot_real = DeclareLaunchArgument(
        "robot_real", default_value=EnvironmentVariable("ROBOT_REAL", default_value="false")
    )
    robot_name = DeclareLaunchArgument("robot_name", description="Name of the robot")

    age_gender_use_gpu = DeclareLaunchArgument("age_gender_use_gpu", default_value=LaunchConfiguration("robot_real"))

    color_extractor_srv_name = DeclareLaunchArgument("color_extractor_srv_name", default_value="extract_color")
    face_recognition_srv_name = DeclareLaunchArgument(
        "face_recognition_srv_name", default_value="face_recognition/recognize"
    )
    pose_estimation_srv_name = DeclareLaunchArgument(
        "pose_estimation_srv_name",
        default_value=["/", LaunchConfiguration("robot_name"), TextSubstitution(text="/pose_estimation/recognize")],
    )
    face_properties_srv_name = DeclareLaunchArgument(
        "face_properties_srv_name", default_value="face_recognition/get_face_properties"
    )

    enable_age_gender_detection = DeclareLaunchArgument("enable_age_gender_detection", default_value="true")
    enable_shirt_color_extraction = DeclareLaunchArgument("enable_shirt_color_extraction", default_value="true")

    enable_topic_mode = DeclareLaunchArgument("enable_topic_mode", default_value="false")
    head_rgbd_sensor = ["/", LaunchConfiguration("robot_name"), TextSubstitution(text="/head_rgbd_sensor")]
    camera_info_depth = DeclareLaunchArgument(
        "camera_info_depth", default_value=[*head_rgbd_sensor, TextSubstitution(text="/depth_registered/camera_info")]
    )
    image_depth = DeclareLaunchArgument(
        "image_depth", default_value=[*head_rgbd_sensor, TextSubstitution(text="/depth_registered/image")]
    )
    image_rgb = DeclareLaunchArgument(
        "image_rgb", default_value=[*head_rgbd_sensor, TextSubstitution(text="/rgb/image_raw")]
    )

    face_recognition_db = DeclareLaunchArgument("face_recognition_db", default_value="")
    face_recognition_config = DeclareLaunchArgument(
        "face_recognition_config",
        default_value=PathJoinSubstitution(
            [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "world_modeling", "face_recognition.yaml"]
        ),
    )
    age_gender_model = DeclareLaunchArgument(
        "age_gender_model", default_value="~/data/pytorch_models/best-epoch47-0.9314.onnx"
    )

    people_recognition_3d_config = DeclareLaunchArgument(
        "people_recognition_3d_config",
        default_value=PathJoinSubstitution([FindPackageShare("people_recognition_3d"), "config", "config.yaml"]),
    )

    real = IfCondition(LaunchConfiguration("robot_real"))
    simulation = UnlessCondition(LaunchConfiguration("robot_real"))
    training_data = ["~/MEGA/data/", LaunchConfiguration("robot_env"), TextSubstitution(text="/training_data")]

    # People recognition
    people_recognition = GroupAction(
        [
            PushRosNamespace("people_recognition"),
            # People recognition 3D
            Node(
                package="people_recognition_3d",
                executable="people_recognition_3d_node",
                name="people_recognition_3d_node",
                output="log",
                condition=real,
                parameters=[
                    ParameterFile(LaunchConfiguration("people_recognition_3d_config"), allow_substs=True),
                    {"enable_topic_mode": ParameterValue(LaunchConfiguration("enable_topic_mode"), value_type=bool)},
                ],
                remappings=[
                    ("camera_info_depth", LaunchConfiguration("camera_info_depth")),
                    ("depth", LaunchConfiguration("image_depth")),
                    ("rgb", LaunchConfiguration("image_rgb")),
                ],
            ),
            Node(
                package="people_recognition_3d",
                executable="dummy_people_recognition_3d_node.py",
                name="people_recognition_3d_node",
                output="log",
                condition=simulation,
            ),
            # People recognition 2D
            Node(
                package="people_recognition_2d",
                executable="people_recognition_2d_node",
                name="people_recognition_2d_node",
                output="log",
                parameters=[
                    {
                        "color_extractor_srv_name": LaunchConfiguration("color_extractor_srv_name"),
                        "face_recognition_srv_name": LaunchConfiguration("face_recognition_srv_name"),
                        "pose_estimation_srv_name": LaunchConfiguration("pose_estimation_srv_name"),
                        "face_properties_srv_name": LaunchConfiguration("face_properties_srv_name"),
                        "enable_age_gender_detection": ParameterValue(
                            LaunchConfiguration("enable_age_gender_detection"), value_type=bool
                        ),
                        "enable_shirt_color_extraction": ParameterValue(
                            LaunchConfiguration("enable_shirt_color_extraction"), value_type=bool
                        ),
                    }
                ],
            ),
            # Face recognition
            GroupAction(
                [
                    PushRosNamespace("face_recognition"),
                    Node(
                        package="image_recognition_face_recognition",
                        executable="face_recognition_node",
                        name="face_recognition",
                        output="log",
                        parameters=[
                            ParameterFile(LaunchConfiguration("face_recognition_config"), allow_substs=True),
                            {
                                "topic_save_images": False,
                                "service_save_images": ParameterValue(
                                    LaunchConfiguration("robot_real"), value_type=bool
                                ),
                                "save_images_folder": training_data,
                                "db": LaunchConfiguration("face_recognition_db"),
                            },
                        ],
                    ),
                    # Age gender classification
                    Node(
                        package="image_recognition_age_gender",
                        executable="face_properties_node",
                        name="age_gender_estimation",
                        output="log",
                        parameters=[
                            {
                                "weights_file_path": LaunchConfiguration("age_gender_model"),
                                "save_images": ParameterValue(LaunchConfiguration("robot_real"), value_type=bool),
                                "save_images_folder": training_data,
                                "use_gpu": ParameterValue(LaunchConfiguration("age_gender_use_gpu"), value_type=bool),
                            }
                        ],
                    ),
                ]
            ),
            # Colour extraction
            Node(
                package="image_recognition_color_extractor",
                executable="color_extractor_node",
                name="color_extractor_node",
                output="log",
            ),
        ]
    )

    return LaunchDescription(
        [
            robot_env,
            robot_real,
            robot_name,
            age_gender_use_gpu,
            color_extractor_srv_name,
            face_recognition_srv_name,
            pose_estimation_srv_name,
            face_properties_srv_name,
            enable_age_gender_detection,
            enable_shirt_color_extraction,
            enable_topic_mode,
            camera_info_depth,
            image_depth,
            image_rgb,
            face_recognition_db,
            face_recognition_config,
            age_gender_model,
            people_recognition_3d_config,
            people_recognition,
        ]
    )
