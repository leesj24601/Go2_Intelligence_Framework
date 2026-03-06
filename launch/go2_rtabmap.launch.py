from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 현재 파일이나 터미널 통해 받은 인자를 받아와서 이 파일 내에서 조건문을 나누거나 노드 파라미터로 전달하는 데 사용 
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    slam_db = LaunchConfiguration("slam_db")
    localization_db = LaunchConfiguration("localization_db")   

    # 실제 카메라 토픽과 일치 시키기 위해 리매핑 
    camera_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("depth/image", "/camera/depth/image_rect_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
    ]

    # Static TF 1: base_link → camera_link (위치만, 회전 없음)
    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.30", "0.0", "0.05",
            "0", "0", "0",
            "base_link",
            "camera_link",
        ],
    )

    # Static TF 2: camera_link → camera_optical_frame (회전만, 위치 없음)
    #카메라 센서의 물리적인 위치(camera_link)에서, 영상을 픽셀 단위로 처리하기 위해 축 방향만 90도씩 틀어서 새롭게 camera_optical_frame이라는 기준을 만들어주는 역할
    #카메라 이미지 픽셀 기준: 영상은 평면이므로 오른쪽이 X축, 아래쪽이 Y축이고, 깊이(물력을 바라보는 정면 방향)가 Z축
    camera_to_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "-1.5708", "0", "-1.5708",
            "camera_link",
            "camera_optical_frame",
        ],
    )

    # 공통 파라미터 (database_path는 모드별로 아래 각 노드에서 개별 지정)
    _rtabmap_common_params = {
        "frame_id": "camera_link",
        "map_frame_id": "map",
        "odom_frame_id": "odom",
        "subscribe_depth": True,
        "subscribe_odom_info": False,
        "approx_sync": True,
        "approx_sync_max_interval": 0.5,
        "publish_tf": True,
        "tf_delay": 0.05,
        "wait_for_transform": 0.5,
        "qos": 1,
        "queue_size": 5,
        "use_sim_time": use_sim_time,
        "subscribe_imu": True,
        "Rtabmap/DetectionRate": "0.5",
        "Rtabmap/LoopClosureReextractFeatures": "true",
        "Reg/Strategy": "0",            # Visual 기반
        "Vis/EstimationType": "2",      # 3D-3D: depth로 양쪽 3D 좌표 추출 후 매칭
                                        # 시뮬(완벽한 depth) + 실 로봇(RealSense) 모두 안정적
        "Vis/MinInliers": "15",         # 기본값 20 → 15 (시뮬 텍스처 빈약 보완)
        "RGBD/OptimizeMaxError": "3.0",
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/AngularUpdate": "0.1",
        "RGBD/LinearUpdate": "0.1",
        "Reg/Force3DoF": "false",
        "Grid/FromDepth": "true",
        "Grid/RangeMax": "5.0",
        "Grid/CellSize": "0.05",
        "Grid/MaxGroundHeight": "0.05",
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/NormalsSegmentation": "false",
        "Rtabmap/MemoryThr": "0",
        "Rtabmap/ImageBufferSize": "1",
    }

    # odom 소스:
    #   시뮬: Isaac Sim OmniGraph (IsaacComputeOdometry) → /odom (ground truth)
    #   실로봇: unitree_ros2 LiDAR+IMU 퓨전 → /odom 또는 /utlidar/robot_odom
    #           (연결 후 토픽명 확인 필요, 다르면 아래 remapping 수정)
    _rtabmap_remappings = camera_remappings + [
        ("odom", "/odom"),
        ("imu", "/imu/data"),
    ]

    # SLAM 모드: 맵 생성 (localization=false, 기본값)
    # -d 플래그로 기존 slam_db 초기화 후 새 맵 생성
    rtabmap_slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        condition=UnlessCondition(localization),
        parameters=[{
            **_rtabmap_common_params,
            "database_path": slam_db,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
        }],
        remappings=_rtabmap_remappings,
        arguments=["-d"],  # 기존 DB 삭제 후 새로 시작
    )

    # Localization 모드: 저장된 맵 불러와 위치 추정만 (localization=true)
    # localization_db를 read-only로 열어 맵 발행 + map→odom TF 발행
    rtabmap_localization_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        condition=IfCondition(localization),
        parameters=[{
            **_rtabmap_common_params,
            "database_path": localization_db,
            "Mem/IncrementalMemory": "false",      # 새 노드 추가 안 함
            "Mem/InitWMWithAllNodes": "true",       # DB 전체 로드
            "Rtabmap/DetectionRate": "2.0",         # 0.5 → 2.0Hz (빠른 재탐지)
            "RGBD/LinearUpdate": "0.0",             # 정지 중에도 처리
            "RGBD/AngularUpdate": "0.0",            # 정지 중에도 처리
            "Rtabmap/LoopThr": "0.11",              # 정상 임계값으로 복구 (0.11 = 11%)
            "Kp/MaxFeatures": "1000",               # 특징점 수 2배 (500 → 1000)
        }],
        remappings=_rtabmap_remappings,
        arguments=[],  # -d 없음 → 기존 DB 유지
    )

    # Phase 2: Depth → LaserScan 변환
    # 카메라 위치: base_link 기준 x=0.30, z=0.05(지면에서 약 0.33m)
    # 이미지 해상도: 240×320, 중앙 행(row 120) 근처가 수평면
    depthimage_to_laserscan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        parameters=[
            {
                "scan_height": 10,       # 중앙 10행 평균 → 수평면 노이즈 감소
                "scan_time": 0.1,        # 10Hz (depth와 동일)
                "range_min": 0.2,
                "range_max": 5.0,
                "output_frame": "camera_link",
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("depth", "/camera/depth/image_rect_raw"),
            ("depth_camera_info", "/camera/camera_info"),
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock from /clock topic",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="Launch in localization mode",
            ),
            DeclareLaunchArgument(
                "slam_db",
                default_value="/home/cvr/Desktop/sj/go2_intelligence_framework/maps/rtabmap.db",
                description="DB path for SLAM mode (overwritten on each run)",
            ),
            DeclareLaunchArgument(
                "localization_db",
                default_value="/home/cvr/Desktop/sj/go2_intelligence_framework/maps/rtabmap_ground_truth.db",
                description="DB path for localization mode (read-only, not overwritten)",
            ),
            base_to_camera_tf,
            camera_to_optical_tf,
            depthimage_to_laserscan,
            rtabmap_slam_node,
            rtabmap_localization_node,
        ]
    )