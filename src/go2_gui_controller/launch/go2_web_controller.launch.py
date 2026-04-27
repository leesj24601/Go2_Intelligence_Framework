from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    waypoint_file = LaunchConfiguration("waypoint_file")
    semantic_object_file = LaunchConfiguration("semantic_object_file")
    semantic_map_id = LaunchConfiguration("semantic_map_id")
    semantic_source_fingerprint = LaunchConfiguration("semantic_source_fingerprint")
    semantic_manifest_policy = LaunchConfiguration("semantic_manifest_policy")
    semantic_manifest_override = LaunchConfiguration("semantic_manifest_override")

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
            DeclareLaunchArgument(
                "semantic_object_file",
                default_value="",
                description="Optional path to a semantic object yaml file.",
            ),
            DeclareLaunchArgument(
                "semantic_map_id",
                default_value="",
                description="Semantic map id expected by the semantic object manifest.",
            ),
            DeclareLaunchArgument(
                "semantic_source_fingerprint",
                default_value="",
                description="Source fingerprint expected by the semantic object manifest.",
            ),
            DeclareLaunchArgument(
                "semantic_manifest_policy",
                default_value="strict",
                description="Semantic manifest validation policy: strict or warn.",
            ),
            DeclareLaunchArgument(
                "semantic_manifest_override",
                default_value="false",
                description="Set true only for diagnostic semantic map runs.",
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
                        "semantic_object_file": semantic_object_file,
                        "semantic_map_id": semantic_map_id,
                        "semantic_source_fingerprint": semantic_source_fingerprint,
                        "semantic_manifest_policy": semantic_manifest_policy,
                        "semantic_manifest_override": semantic_manifest_override,
                    }
                ],
            ),
        ]
    )
