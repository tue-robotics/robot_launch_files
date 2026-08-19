from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Record the topics that are interesting for a RoboCup run, throttled where needed."""
    # HMI Recorder
    hmi = GroupAction(
        [PushRosNamespace("hmi"), Node(package="test_tools", executable="hmi_logger.py", name="hmi_logger")]
    )

    # Topics that are going to be recorded, throttle them if required
    throttle_tf = Node(
        package="topic_tools",
        executable="throttle",
        name="record_throttle_tf",
        arguments=["bytes", "/tf", "100000", "1", "record/tf"],
    )
    throttle_ed_map = Node(
        package="topic_tools",
        executable="throttle",
        name="record_throttle_ed_map",
        arguments=["messages", "ed/navigation/map", "0.05", "record/map"],
    )
    throttle_gmapping_map = Node(
        package="topic_tools",
        executable="throttle",
        name="record_throttle_gmapping_map",
        arguments=["messages", "ed/gmapping/map", "0.05", "record/map"],
    )
    relay_base_plan = Node(
        package="topic_tools",
        executable="relay",
        name="record_relay_base_plan",
        arguments=["local_planner/action_server/goal", "record/base_plan"],
    )
    relay_body_plan = Node(
        package="topic_tools",
        executable="relay",
        name="record_relay_body_plan",
        arguments=["body/joint_trajectory_action/goal", "record/body_plan"],
    )
    throttle_odom = Node(
        package="topic_tools",
        executable="throttle",
        name="record_throttle_odom",
        arguments=["messages", "base/measurements", "5", "record/odom"],
    )
    throttle_base_laser = Node(
        package="topic_tools",
        executable="throttle",
        name="record_throttle_base_laser",
        arguments=["messages", "base_laser/scan", "5", "record/scan"],
    )

    # recorder server
    recorder = Node(package="test_tools", executable="robocup_recorder.py", name="robocup_recorder_server")

    return LaunchDescription(
        [
            hmi,
            throttle_tf,
            throttle_ed_map,
            throttle_gmapping_map,
            relay_base_plan,
            relay_body_plan,
            throttle_odom,
            throttle_base_laser,
            recorder,
        ]
    )
