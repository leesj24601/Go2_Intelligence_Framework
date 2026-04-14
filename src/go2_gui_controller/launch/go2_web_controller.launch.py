from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    waypoint_file = LaunchConfiguration("waypoint_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="Runtime mode: sim or real.",
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value="",
                description="Optional path to a waypoint yaml file.",
            ),
            Node(
                package="go2_gui_controller",
                executable="web_controller",
                name="go2_web_controller",
                output="screen",
                parameters=[
                    {
                        "mode": mode,
                        "waypoint_file": waypoint_file,
                    }
                ],
            ),
        ]
    )
