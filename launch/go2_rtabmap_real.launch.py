from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    localization = LaunchConfiguration("localization")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    restamped_odom_topic = "/utlidar/robot_odom_restamped"
    rgbd_topic = "/camera/rgbd_image"

    odom_restamper = Node(
        package="go2_gui_controller",
        executable="odom_restamper",
        name="odom_restamper",
        output="screen",
        parameters=[{
            "publish_tf": True,
        }],
        remappings=[
            ("input_odom", odom_topic),
            ("output_odom", restamped_odom_topic),
        ],
    )

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        output="screen",
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "queue_size": 30,
            "qos": 1,
        }],
        remappings=[
            ("rgb/image", rgb_topic),
            ("depth/image", depth_topic),
            ("rgb/camera_info", camera_info_topic),
            ("rgbd_image", rgbd_topic),
        ],
    )

    # Static TF 1: base_link → camera_link
    # TODO: RealSense 실제 장착 위치 측정 후 수정 (현재는 시뮬 기준 어림값)
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

    # Static TF 2: camera_link → camera_color_optical_frame
    # RealSense color/aligned-depth topics use camera_color_optical_frame.
    camera_to_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "-1.5708", "0", "-1.5708",
            "camera_link",
            "camera_color_optical_frame",
        ],
    )

    # 공통 파라미터 (실로봇 전용)
    _rtabmap_common_params = {
        "database_path": "/home/cvr/Desktop/sj/go2_intelligence_framework/maps/rtabmap_real.db",
        "frame_id": "camera_link",
        "map_frame_id": "map",
        # Use pre-synchronized RGBD stream and external odom via TF tree.
        "odom_frame_id": "odom",
        "subscribe_depth": False,
        "subscribe_rgbd": True,
        "subscribe_odom_info": False,
        "subscribe_imu": False,            # unitree_ros2 /odom에 IMU 이미 융합됨
        "odom_to_tf": True,                # map->odom TF publish 유지, external odom은 TF로 사용
        "approx_sync": False,              # rgbd_image 단독 입력이므로 추가 sync 불필요
        "publish_tf": True,
        "tf_delay": 0.05,
        "wait_for_transform": 0.5,
        "qos": 1,
        "queue_size": 30,                 # RTAB-Map sync_queue_size
        "topic_queue_size": 30,           # 각 입력 토픽 message_filters 버퍼도 함께 확대
        "use_sim_time": False,
        "Rtabmap/DetectionRate": "1.0",    # 시뮬 0.5Hz → 실로봇 1.0Hz
        "Rtabmap/LoopClosureReextractFeatures": "true",
        "Reg/Strategy": "0",               # Visual 기반
        "Vis/EstimationType": "2",         # 3D-3D
        "Vis/MinInliers": "20",            # 시뮬 보정값 15 → 기본값 20 복원
        "RGBD/OptimizeMaxError": "3.0",
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/AngularUpdate": "0.1",
        "RGBD/LinearUpdate": "0.1",
        "Reg/Force3DoF": "true",
        "Grid/FromDepth": "true",
        "Grid/RangeMax": "4.0",            # D435i 실용 범위 (시뮬 5.0 → 4.0)
        "Grid/CellSize": "0.05",
        "Grid/MaxGroundHeight": "0.05",
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/NormalsSegmentation": "false",
        "Rtabmap/MemoryThr": "0",
        "Rtabmap/ImageBufferSize": "1",
    }

    _rtabmap_remappings = [
        ("rgbd_image", rgbd_topic),
    ]

    # SLAM 모드 (localization=false, 기본값)
    rtabmap_slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        condition=UnlessCondition(localization),
        parameters=[{
            **_rtabmap_common_params,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
        }],
        remappings=_rtabmap_remappings,
        arguments=["-d"],
    )

    # Localization 모드 (localization=true)
    rtabmap_localization_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        condition=IfCondition(localization),
        parameters=[{
            **_rtabmap_common_params,
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",
            "Rtabmap/DetectionRate": "2.0",
            "RGBD/LinearUpdate": "0.0",
            "RGBD/AngularUpdate": "0.0",
            "Rtabmap/LoopThr": "0.11",
            "Kp/MaxFeatures": "1000",
        }],
        remappings=_rtabmap_remappings,
        arguments=[],
    )

    # Depth → LaserScan 변환
    # 현재 실기 설정: 424×240, 요청 15Hz / 실측 aligned depth 약 13Hz
    # scan_height: 10행 (240 기준 중앙 약 4%) → 시뮬 설정과 같은 비율
    depthimage_to_laserscan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        parameters=[{
            "scan_height": 10,       # 240행 기준 중앙 10행 사용
            "scan_time": 0.08,       # 실측 depth 주기(~76ms)에 맞춰 보정
            "range_min": 0.3,        # D435i 최소 거리 (시뮬 0.2 → 0.3)
            "range_max": 4.0,        # D435i 실용 범위 (시뮬 5.0 → 4.0)
            "output_frame": "camera_link",
            "use_sim_time": False,
        }],
        remappings=[
            ("depth", depth_topic),
            ("depth_camera_info", camera_info_topic),
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="SLAM 모드=false / Localization 모드=true",
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
            description="Camera info topic matching the RGB stream.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/utlidar/robot_odom",
            description="Raw odometry topic published by the robot bridge.",
        ),
        odom_restamper,
        rgbd_sync,
        base_to_camera_tf,
        camera_to_optical_tf,
        depthimage_to_laserscan,
        rtabmap_slam_node,
        rtabmap_localization_node,
    ])
