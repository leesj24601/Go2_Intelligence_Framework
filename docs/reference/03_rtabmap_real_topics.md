---
tags: [project, rtabmap, topics, real-robot]
status: completed
project: go2_intelligence_framework
type: reference
created: 2026-03-25
---

# RTAB-Map Real 실행 시 입출력 토픽 정리

기준 파일:

- `launch/go2_rtabmap_real.launch.py`
- `src/go2_gui_controller/go2_gui_controller/odom_restamper.py`

검증 방법:

- 코드 기준 정적 분석
- 2026-03-25에 `ros2 launch launch/go2_rtabmap_real.launch.py` 실행 후 `ros2 node info`로 런타임 인터페이스 확인

주의:

- 현재 코드 기본값은 `odom_restamper_publish_tf:=false`이다.
- 따라서 **기본 실행에서는 `odom -> base_link` TF를 upstream odom publisher가 제공하는 것을 전제**한다.
- 아래 문서에서 `odom_restamper`가 `/tf`를 발행한다고 적힌 부분은
  - 과거 런타임 관측 결과이거나
  - `odom_restamper_publish_tf:=true`로 켠 경우에 해당한다.

## 1. 결론 요약

현재 `go2_rtabmap_real.launch.py` 기본 실행은 `localization:=false` 이므로 **SLAM(맵 생성) 모드**이다.

참고:

- `launch/go2_navigation_real.launch.py` 에서는 같은 `go2_rtabmap_real.launch.py`를 포함하되 기본값을 `localization:=true`로 넘긴다.
- 따라서 **입출력 토픽 구조는 거의 같고**, 메모리 동작만 localization 모드로 바뀐다고 보면 된다.

운영 기준:

- 현재 실운영은 **Go2에서 제공되는 aligned depth**를 그대로 사용한다.
- 즉 기본 카메라 입력 기준은
  - `/camera/color/image_raw`
  - `/camera/aligned_depth_to_color/image_raw`
  - `/camera/color/camera_info`
  이다.
- 실측 기준 `align_depth.enable:=false`일 때 Go2 내부 raw `color/depth`는 약 `15Hz`로 안정적이다.
- `align_depth.enable:=true` 운영에서는 Go2 내부 `color/aligned_depth`가 약 `10Hz`, PC 수신 aligned depth는 약 `8Hz` 수준으로 떨어진다.
- 현재 Go2 내부 병목은 `align_depth` 경로로 보며, 세부 후보는 `align` 연산 자체와 color/depth를 맞춰 기다리는 `sync` 동작이다.
- 따라서 현재 실운영에서 카메라 관련 시간 파라미터는 요청 fps가 아니라 **PC 실수신 aligned depth 약 8Hz** 기준으로 잡는 것이 맞다.
- [[04_real_robot_deploy]]에 남아 있는 `align_depth:=false + raw depth/PC 정렬` 경로는 현재 기본 운영 경로가 아니라 과거 트러블슈팅 기록으로 봐야 한다.

맵 생성에 실제로 핵심적으로 쓰이는 입력은 아래 2가지다.

1. **카메라 RGB-D 데이터**
   - 원본 입력:
     - `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
     - `/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/msg/Image`)
     - `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
   - `rgbd_sync`가 이를 묶어서
     - `/camera/rgbd_image` (`rtabmap_msgs/msg/RGBDImage`)
     로 재발행
   - RTAB-Map 본체는 실제로 이 `/camera/rgbd_image`를 구독해서 맵을 만든다.

2. **외부 odom/TF**
   - 원본 odom:
     - `/utlidar/robot_odom` (`nav_msgs/msg/Odometry`)
   - `odom_restamper`가 planar 보정 후
     - `/utlidar/robot_odom_restamped`
     를 발행
   - `odom -> base_link` TF는 기본값 기준 upstream odom publisher가 제공하고,
     `odom_restamper_publish_tf:=true`일 때만 `odom_restamper`가 재발행한다.
   - RTAB-Map 본체는 odom 토픽을 직접 구독하지 않고, 내부 TF listener를 통해 `/tf`, `/tf_static`을 사용한다.

중요:

- 현재 설정에서 `depthimage_to_laserscan`이 `/scan`을 발행하지만, RTAB-Map은 `subscribe_scan=false`라서 **이 `/scan`으로 맵을 만들지 않는다.**
- 즉 현재 RTAB-Map 실로봇 맵 생성은 **RealSense RGB-D + LiDAR/IMU 기반 외부 odom(TF)** 조합이다.

## 2. 전체 데이터 흐름

```text
/camera/color/image_raw                \
/camera/aligned_depth_to_color/image_raw ---> /rgbd_sync ---> /camera/rgbd_image ---> /rtabmap
/camera/color/camera_info             /

/utlidar/robot_odom ---> upstream TF (default) or /odom_restamper TF (optional) ---> internal TF listener ---> /rtabmap

/static tf
base_link -> camera_link
camera_link -> camera_color_optical_frame
```

결과적으로 RTAB-Map은 아래 TF 체인을 보고 카메라 pose를 계산한다.

```text
map -> odom -> base_link -> camera_link -> camera_color_optical_frame
```

여기서:

- `map -> odom` 은 RTAB-Map이 발행
- `odom -> base_link` 는 기본값 기준 upstream odom publisher가 발행
- `odom_restamper_publish_tf:=true`이면 `odom_restamper`가 같은 TF를 재발행할 수 있음
- `base_link -> camera_link` 는 정적 TF
- `camera_link -> camera_color_optical_frame` 는 정적 TF

## 3. 맵 생성에 쓰이는 입력 토픽과 데이터

### 3.1 카메라 입력

#### `/camera/color/image_raw`

- 타입: `sensor_msgs/msg/Image`
- 의미: RGB 컬러 프레임
- 사용처: `rgbd_sync`
- RTAB-Map 관점: 특징점 추출, 장소 인식, loop closure, 시각 정합의 입력

#### `/camera/aligned_depth_to_color/image_raw`

- 타입: `sensor_msgs/msg/Image`
- 의미: RGB 프레임과 픽셀 정렬된 depth 이미지
- 사용처: `rgbd_sync`, `depthimage_to_laserscan`
- RTAB-Map 관점: RGB 특징점의 3D 위치 복원, 점군/occupancy grid 생성의 입력

#### `/camera/color/camera_info`

- 타입: `sensor_msgs/msg/CameraInfo`
- 의미: 카메라 내부 파라미터(K, P), 왜곡계수, 해상도, optical frame 정보
- 사용처: `rgbd_sync`, `depthimage_to_laserscan`
- RTAB-Map 관점: RGB와 depth를 올바른 3D 관측으로 해석하는 데 필요

#### `/camera/rgbd_image`

- 타입: `rtabmap_msgs/msg/RGBDImage`
- 발행 노드: `rgbd_sync`
- 구독 노드: `rtabmap`
- 의미: RGB 이미지 + depth 이미지 + camera info를 한 메시지로 동기화한 RTAB-Map 입력
- 런타임 확인:
  - `rgbd_sync subscribed to: /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw, /camera/color/camera_info`
  - `rtabmap subscribed to: /camera/rgbd_image`

### 3.2 Odometry / TF 입력

#### `/utlidar/robot_odom`

- 타입: `nav_msgs/msg/Odometry`
- 현재 실측 헤더:
  - `header.frame_id: odom`
  - `child_frame_id: base_link`
- 의미: Unitree/utlidar 쪽에서 제공하는 외부 odometry
- 프로젝트 문서 해석:
  - 실로봇에서는 `LiDAR + IMU`가 짧은 구간 상대 이동을 제공하고,
  - RTAB-Map은 카메라 관측으로 정합 및 loop closure를 수행

#### `odom_restamper`가 실제로 하는 일

`src/go2_gui_controller/go2_gui_controller/odom_restamper.py` 기준:

- 입력 `Odometry`를 받아 timestamp를 현재 clock 기준으로 보정
- `z` 위치를 `0.0`으로 강제
- roll/pitch를 제거하고 yaw만 남긴 quaternion으로 교체
- `linear.z`, `angular.x`, `angular.y`를 `0.0`으로 보정
- 보정된 결과를 `/utlidar/robot_odom_restamped`로 재발행
- `publish_tf=true`일 때만 같은 pose로 `/tf`에 transform도 발행

즉, 걷기 진동으로 생기는 z/roll/pitch 흔들림을 줄여 실내 평면 운용에서 더 안정적인 2D odom을 만드는 역할이다.

현재 운영 해석:

- **RTAB-Map 본체는 `/utlidar/robot_odom_restamped`를 직접 쓰지 않는다.**
- RTAB-Map은 기본값 기준 upstream가 발행하는 `odom -> base_link` TF를 사용한다.
- `/utlidar/robot_odom_restamped`는 현재 **Nav2 쪽에서 raw odom의 과거 timestamp(2024년 문제)를 피하기 위한 소비 토픽**으로 보는 것이 맞다.
- 따라서 현재 planarization은 **SLAM 본체의 자세 추정 경로**가 아니라 **Nav2용 2D odom 경로**에 주로 적용된다고 이해하는 것이 맞다.
- 다만 이 처리 방식은 **실내 평면/완만한 바닥**을 전제로 한 타협이다.
- 경사로, 요철, 3D 자세 추정, 자세 안정화처럼 roll/pitch가 실제 의미를 가지는 소비자에는 raw `/utlidar/robot_odom` 또는 별도 3D 추정 경로를 써야 한다.

#### `/tf`, `/tf_static`

- 타입: `tf2_msgs/msg/TFMessage`
- RTAB-Map 본체는 `ros2 node info /rtabmap`에 직접 보이지 않지만,
  내부 `transform_listener_impl_*` 노드가 `/tf`, `/tf_static`을 구독한다.
- RTAB-Map 관련 TF 구성:
  - `/tf`: 기본값 기준 upstream odom publisher가 `odom -> base_link` 발행
  - `/tf`: `odom_restamper_publish_tf:=true`이면 `odom_restamper`도 같은 TF를 재발행 가능
  - `/tf_static`: `base_link -> camera_link`, `camera_link -> camera_color_optical_frame`

### 3.3 현재 맵 생성에 쓰이지 않는 보조 입력

#### `/scan`

- 타입: `sensor_msgs/msg/LaserScan`
- 발행 노드: `depthimage_to_laserscan`
- 생성 방식:
  - `/camera/aligned_depth_to_color/image_raw`
  - `/camera/color/camera_info`
  를 이용해 depth 중앙 10행만 잘라 2D scan으로 변환
- 현재 RTAB-Map 설정:
  - `subscribe_scan = false`
  - `subscribe_scan_cloud = false`

따라서 `/scan`은 현재 RTAB-Map 맵 생성 입력이 아니다.
이 토픽은 Nav2 또는 디버깅/RViz용 보조 데이터로 보는 것이 맞다.

## 4. 현재 RTAB-Map 노드가 구독하는 토픽

`ros2 node info /rtabmap` 기준 구독 토픽은 아래와 같다.

### 4.1 맵 생성에 실질적으로 쓰이는 핵심 입력

- `/camera/rgbd_image` (`rtabmap_msgs/msg/RGBDImage`)
- 내부 TF listener를 통한 `/tf`, `/tf_static`

### 4.2 RTAB-Map이 인터페이스상 열어 두는 선택 입력

아래는 현재 노드 인터페이스에 존재하지만, 현재 런치 구성상 맵 생성 핵심 입력으로 연결된 것은 아니다.

- `/global_pose`
- `/goal`
- `/goal_node`
- `/initialpose`
- `/imu`
- `/gps/fix`
- `/apriltag/detections`
- `/tag_detections`
- `/aruco/detections`
- `/aruco_opencv/detections`
- `/landmark_detection`
- `/landmark_detections`
- `/user_data_async`
- `/rtabmap/republish_node_data`

주의:

- `ros2 node info`에는 optional subscriber도 함께 나타난다.
- 하지만 현재 launch 파라미터는
  - `subscribe_rgbd = true`
  - `subscribe_depth = false`
  - `subscribe_imu = false`
  - `subscribe_odom_info = false`
  - `subscribe_scan = false`
  로 설정되어 있다.

## 5. 현재 RTAB-Map 노드가 발행하는 토픽

`ros2 node info /rtabmap` 기준.

### 5.1 맵/위치추정 핵심 출력

- `/map` (`nav_msgs/msg/OccupancyGrid`)
  - 2D occupancy map
- `/mapData` (`rtabmap_msgs/msg/MapData`)
  - RTAB-Map 내부 노드/센서 데이터 포함 맵 데이터
- `/mapGraph` (`rtabmap_msgs/msg/MapGraph`)
  - pose graph
- `/mapOdomCache` (`rtabmap_msgs/msg/MapGraph`)
  - map-odom 관계 캐시
- `/localization_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
  - RTAB-Map 기준 localization 결과
- `/tf` (`tf2_msgs/msg/TFMessage`)
  - `map -> odom` TF

### 5.2 3D / local grid / octomap 출력

- `/cloud_map`
- `/cloud_ground`
- `/cloud_obstacles`
- `/local_grid_empty`
- `/local_grid_ground`
- `/local_grid_obstacle`
- `/octomap_binary`
- `/octomap_full`
- `/octomap_empty_space`
- `/octomap_ground`
- `/octomap_obstacles`
- `/octomap_occupied_space`
- `/octomap_global_frontier_space`
- `/octomap_grid`
- `/grid_prob_map`

이들은 depth 기반 3D/2D 공간 표현 결과다.

### 5.3 경로/디버그/부가 출력

- `/info`
- `/labels`
- `/landmarks`
- `/global_path`
- `/global_path_nodes`
- `/local_path`
- `/local_path_nodes`
- `/mapPath`
- `/goal_out`
- `/goal_reached`
- `/diagnostics`

## 6. 이 launch 전체가 발행하는 주요 토픽

`go2_rtabmap_real.launch.py` 전체 관점에서 보면 RTAB-Map 외 보조 노드까지 포함하여 아래 토픽이 새로 만들어진다.

### 6.1 보조 노드 출력

- `/camera/rgbd_image` (`rgbd_sync`)
- `/rgbd_image/compressed` (`rgbd_sync`)
- `/utlidar/robot_odom_restamped` (`odom_restamper`)
- `/tf` (`upstream odom publisher`, `rtabmap`)
- `/tf` (`odom_restamper`, optional when enabled)
- `/tf_static` (정적 TF 2개)
- `/scan` (`depthimage_to_laserscan`)

### 6.2 RTAB-Map 출력

- `/map`
- `/mapData`
- `/mapGraph`
- `/mapOdomCache`
- `/localization_pose`
- `/info`
- `/cloud_*`
- `/local_grid_*`
- `/octomap_*`
- `/global_path`, `/local_path`, `/mapPath`

## 7. 코드 근거

### 7.1 `launch/go2_rtabmap_real.launch.py`

핵심 근거:

- `rgbd_sync` 입력 remap
  - `rgb/image <- rgb_topic`
  - `depth/image <- depth_topic`
  - `rgb/camera_info <- camera_info_topic`
  - `rgbd_image -> /camera/rgbd_image`
- RTAB-Map 파라미터
  - `frame_id = camera_link`
  - `odom_frame_id = odom`
  - `subscribe_depth = False`
  - `subscribe_rgbd = True`
  - `subscribe_imu = False`
  - `odom_to_tf = True`
  - `publish_tf = True`
- `odom_restamper` launch 인자
  - `odom_restamper_publish_tf = false` (기본값)
- RTAB-Map remap
  - `rgbd_image <- /camera/rgbd_image`
- LaserScan 변환
  - `depth <- depth_topic`
  - `depth_camera_info <- camera_info_topic`
  - `scan -> /scan`

### 7.2 `odom_restamper.py`

핵심 근거:

- 입력 odom 구독 후 pose/twist를 복사
- `position.z = 0.0`
- orientation은 yaw만 남기고 재구성
- 보정된 odom을 publish
- `publish_tf=true`이면 같은 값을 `TransformStamped`로 `/tf`에 publish

## 8. 한 줄 정리

현재 `rtabmap real` 맵 생성은 **`/camera/rgbd_image`(RealSense RGB + aligned depth + camera info)** 와 **`/tf` 기반 외부 odom(`odom -> base_link`)** 을 사용하며, 대표적으로 **`/map`**, **`/mapData`**, **`/mapGraph`**, **`/localization_pose`**, **`/tf(map -> odom)`**, 그리고 각종 **`cloud/local_grid/octomap`** 토픽을 발행한다.
