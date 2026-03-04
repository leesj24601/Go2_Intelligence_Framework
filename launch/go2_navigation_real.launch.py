import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PROJECT_DIR = os.path.dirname(_THIS_DIR)


def generate_launch_description():
    localization = LaunchConfiguration("localization")

    nav2_params_file = os.path.join(_PROJECT_DIR, "config", "go2_nav2_params_real.yaml")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # [카메라는 Go2 내부에서 실행]
    # Go2 SSH 접속 후 아래 순서로 실행:
    #
    #   터미널1 (Noetic):
    #     source /opt/ros/noetic/setup.bash
    #     roslaunch realsense2_camera rs_camera.launch \
    #       camera:=camera \
    #       depth_width:=640 depth_height:=480 depth_fps:=15 \
    #       color_width:=640 color_height:=480 color_fps:=15 \
    #       align_depth:=true enable_infra1:=false enable_infra2:=false
    #
    #   터미널2 (Foxy):
    #     source /opt/ros/foxy/setup.bash
    #     source ~/ros1_bridge_ws/install/setup.bash
    #     ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    #
    # 이후 노트북에서 이 런치 파일 실행

    # RTAB-Map (실로봇 전용)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_THIS_DIR, "go2_rtabmap_real.launch.py")
        ),
        launch_arguments={
            "localization": localization,
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
            "params_file": nav2_params_file,
            "map_subscribe_transient_local": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="SLAM 모드=false / Localization 모드=true",
        ),
        rtabmap_launch,
        nav2_launch,
    ])
