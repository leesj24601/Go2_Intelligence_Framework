from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")

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

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        output="screen",
        parameters=[
            {
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
                # IMU 구독
                "subscribe_imu": True,
                # RTAB-Map 파라미터
                "Rtabmap/DetectionRate": "0.5",
                "Rtabmap/LoopClosureReextractFeatures": "true",
                "Reg/Strategy": "0",
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
                # Localization 모드일 때 매핑 중지 (Read-only 모드)
                "Mem/IncrementalMemory": "false" if localization == "true" else "true",
                "Mem/InitWMWithAllNodes": "true" if localization == "true" else "false",
            }
        ],
        remappings=camera_remappings + [
            ("odom", "/odom"),
            ("imu", "/imu/data"),
        ],
        # Mapping 모드일 때만 -d(삭제) 옵션 추가
        arguments=["-d"] if localization == "false" else [],
    )

    # Phase 2: Depth → LaserScan 변환
    # 카메라 위치: base_link 기준 x=0.30, z=0.05 (지면에서 약 0.33m)
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
            base_to_camera_tf,
            camera_to_optical_tf,
            depthimage_to_laserscan,
            rtabmap_node,
        ]
    )
