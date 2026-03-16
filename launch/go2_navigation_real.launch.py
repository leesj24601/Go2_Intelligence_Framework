import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PROJECT_DIR = os.path.dirname(_THIS_DIR)


def generate_launch_description():
    localization = LaunchConfiguration("localization")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    nav2_params_file = os.path.join(_PROJECT_DIR, "config", "go2_nav2_params_real.yaml")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": "false",
            "odom_topic": odom_topic,
        },
        convert_types=True,
    )

    # [카메라는 Go2 내부에서 ROS2로 직접 실행]
    # 확인된 RealSense 실행 예시:
    #
    #   ros2 launch realsense2_camera rs_launch.py \
    #     depth_module.profile:=424x240x30 \
    #     rgb_camera.profile:=424x240x30 \
    #     enable_infra1:=false \
    #     enable_infra2:=false \
    #     align_depth.enable:=true
    #
    # 이 경우 기본 토픽은 /camera/... 네임스페이스를 사용한다.

    # RTAB-Map (실로봇 전용)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_THIS_DIR, "go2_rtabmap_real.launch.py")
        ),
        launch_arguments={
            "localization": localization,
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "odom_topic": odom_topic,
        }.items(),
    )

    # Nav2: bt_navigator, planner, controller, behavior 등
    # map_server/amcl 제외 — RTAB-Map이 map→odom TF + /map 토픽 직접 발행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": configured_nav2_params,
            "map_subscribe_transient_local": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization",
            default_value="true",
            description="Use localization mode for the RTAB-Map stack during navigation",
        ),
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/camera/color/image_raw",
            description="RGB image topic from the real camera bridge.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/aligned_depth_to_color/image_raw",
            description="Aligned depth image topic from the real camera bridge.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
            description="Camera info topic aligned with the RGB stream.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/utlidar/robot_odom",
            description="Odometry topic published by the robot bridge.",
        ),
        rtabmap_launch,
        nav2_launch,
    ])
