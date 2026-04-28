from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 현재 파일이나 터미널 통해 받은 인자를 받아와서 이 파일 내에서 조건문을 나누거나 노드 파라미터로 전달하는 데 사용 
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    slam_db = LaunchConfiguration("slam_db")
    localization_db = LaunchConfiguration("localization_db")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_joint_states = LaunchConfiguration("use_fake_joint_states")

    # ROS 쪽에서 가장 흔한 패턴인 xacro -> robot_state_publisher 흐름을 사용한다.
    # camera_link 체인은 기존 static TF를 유지하므로 본체/다리 모델만 URDF에서 가져온다.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "publish_frequency": 100.0,
                "frame_prefix": "",
            }
        ],
    )

    # joint_state_publisher는 /robot_description 토픽을 구독해 기본 자세 joint_states를 발행한다.
    # 이 방식이 standalone 검증에서 실제로 동작했고, ROS 커뮤니티에서도 흔히 쓰는 패턴이다.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_fake_joint_states),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "rate": 30.0,
            }
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
        "Rtabmap/LoopClosureReextractFeatures": "false",
        "Reg/Strategy": "0",            # Visual 기반
        "Vis/EstimationType": "2",      # 3D-3D: depth로 양쪽 3D 좌표 추출 후 매칭
                                        # 시뮬(완벽한 depth) + 실 로봇(RealSense) 모두 안정적
        "Vis/MinInliers": "15",         # 기본값 20 → 15 (시뮬 텍스처 빈약 보완)
        "RGBD/OptimizeMaxError": "3.0",
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/AngularUpdate": "0.1",
        "RGBD/LinearUpdate": "0.1",
        "Reg/Force3DoF": "true",        # 평지 실내 맵 생성/위치추정에서 roll/pitch 기울기 영향 억제
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
            "Rtabmap/DetectionRate": "1.0",   # 424x240 맵 생성 시 처리 간격 축소
            "Kp/MaxFeatures": "1000",         # 저해상도에서 특징점 부족 완화
            "Rtabmap/LoopThr": "0.35",        # 반복 구조 오인식은 줄이고 localization 후보는 유지
            "RGBD/ProximityBySpace": "false", # 시뮬(gt odom) 맵 생성 시 근접 제약으로 그래프가 꼬이는 현상 완화
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
            "Vis/MinInliers": "15",                 # localization 재탐지 기준. 반복 구조 오인식을 줄이기 위해 8에서 복구
            "Vis/MinInliersDistribution": "0.0",    # 특징 적은 복도/벽면에서 초기 localization이 막히지 않도록 분포 조건 비활성화
            "RGBD/OptimizeMaxError": "3.0",
            "Rtabmap/LoopClosureReextractFeatures": "true",
            "Rtabmap/LoopThr": "0.11",              # 기본 localization 후보 수락 기준
            "RGBD/AggressiveLoopThr": "0.08",       # 초기 위치 획득은 허용하되 기본값(0.05)보다는 약간 보수화
            "Kp/MaxFeatures": "1000",               # 특징점 수 2배 (500 → 1000)
        }],
        remappings=_rtabmap_remappings,
        arguments=[],  # -d 없음 → 기존 DB 유지
    )

    # Phase 2: Depth → LaserScan 변환
    # 카메라 위치: base_link 기준 x=0.30, z=0.05 (지면에서 약 0.33m)
    # 이미지 해상도: 240×320, 중앙 행 (row 120) 근처가 수평면
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
                "description_package",
                default_value="go2_description",
                description="Package that provides the Go2 URDF/Xacro files",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="go2_description.urdf",
                description="URDF/Xacro file used for RViz RobotModel",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="Optional TF/joint prefix for robot description",
            ),
            DeclareLaunchArgument(
                "use_fake_joint_states",
                default_value="false",
                description="Use joint_state_publisher for a fixed pose when no real /joint_states source exists",
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
                default_value="/home/cvr/Desktop/sj/go2_intelligence_framework/maps/rtabmap_office.db",
                description="DB path for localization mode (read-only, not overwritten)",
            ),
            robot_state_publisher,
            joint_state_publisher,
            base_to_camera_tf,
            camera_to_optical_tf,
            depthimage_to_laserscan,
            rtabmap_slam_node,
            rtabmap_localization_node,
        ]
    )
