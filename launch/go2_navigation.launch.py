import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 이 launch 파일의 디렉토리 기준으로 경로 설정
_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_PROJECT_DIR = os.path.dirname(_THIS_DIR)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization_db = LaunchConfiguration("localization_db")

    nav2_params_file = os.path.join(_PROJECT_DIR, "config", "go2_nav2_params.yaml")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup") # nav2 패키지 ON

    # RTAB-Map: localization 모드 (맵 생성 X, 위치 추정만)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_THIS_DIR, "go2_rtabmap.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "localization": "true",
            "localization_db": localization_db,
        }.items(),
    )

    # Nav2: navigation_launch.py (bt_navigator, planner, controller, behavior 등)
    # map_server/amcl 제외 — RTAB-Map이 map→odom TF + /map 토픽 직접 발행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "map_subscribe_transient_local": "true", # Nav2에 필요한 맵(/map)을 가져올 때 요청이 늦더라도 가능 (이전 데이터라도 가져옴)
        }.items(),
    )

    # 위 부분은 설계 코드 , 아래는 실제 실행을 의미 
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock from /clock topic",
            ),
            DeclareLaunchArgument(
                "localization_db",
                default_value=os.path.join(_PROJECT_DIR, "maps", "rtabmap_office.db"),
                description="RTAB-Map database path used for localization during Nav2.",
            ),
            rtabmap_launch,
            nav2_launch,
        ]
    )
