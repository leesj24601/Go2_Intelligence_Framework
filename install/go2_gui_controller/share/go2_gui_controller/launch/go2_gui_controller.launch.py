from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_file = LaunchConfiguration("waypoint_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoint_file",
                default_value="",
                description="Optional path to a waypoint yaml file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time.",
            ),
            Node(
                package="go2_gui_controller",
                executable="gui_controller",
                name="go2_gui_controller",
                output="screen",
                parameters=[
                    {
                        "waypoint_file": waypoint_file,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
