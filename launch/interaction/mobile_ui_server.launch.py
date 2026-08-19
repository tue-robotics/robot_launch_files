from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    """Serve the mobile UI and expose ROS to it over a websocket."""
    # Launch the websocket server (communication via roslib)
    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="log",
        parameters=[{"authenticate": False, "port": 9090, "address": ""}],
    )

    # ROS API: required to get parameters using ROSLIB
    rosapi = Node(package="rosapi", executable="rosapi_node", name="rosapi", output="log")

    # Start the simple webserver to handle static files
    webserver = Node(
        package="tue_mobile_ui",
        executable="webserver.py",
        name="webserver",
        output="log",
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [EnvironmentVariable("ROBOT_BRINGUP_PATH"), "parameters", "interaction", "mobile_ui_server.yaml"]
                ),
                allow_substs=True,
            )
        ],
    )

    return LaunchDescription([rosbridge_websocket, rosapi, webserver])
