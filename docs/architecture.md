# Go2 Intelligence Framework — 전체 구조 문서

> **대상 버전**: ROS 2 Humble · Isaac Sim 5.1.0 · Isaac Lab v2.3.0

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [디렉토리 구조](#2-디렉토리-구조)
3. [주요 모듈 설명](#3-주요-모듈-설명)
   - [3.1 시뮬레이션 환경 (scripts/)](#31-시뮬레이션-환경-scripts)
   - [3.2 SLAM 스택 (launch/go2\_rtabmap\*.launch.py)](#32-slam-스택-launchgo2_rtabmapLaunchpy)
   - [3.3 내비게이션 스택 (launch/go2\_navigation\*.launch.py)](#33-내비게이션-스택-launchgo2_navigationLaunchpy)
   - [3.4 GUI 컨트롤러 (src/go2\_gui\_controller/)](#34-gui-컨트롤러-srcgo2_gui_controller)
   - [3.5 ROS 패키지 의존성 (src/go2\_project\_dependencies/)](#35-ros-패키지-의존성-srcgo2_project_dependencies)
4. [실행 흐름 (시뮬레이션)](#4-실행-흐름-시뮬레이션)
5. [실행 흐름 (실 로봇)](#5-실행-흐름-실-로봇)
6. [ROS 토픽 맵](#6-ros-토픽-맵)
7. [TF 프레임 트리](#7-tf-프레임-트리)
8. [주요 클래스 & 파일 참조](#8-주요-클래스--파일-참조)
9. [의존성 요약](#9-의존성-요약)
10. [개발 로드맵 현황](#10-개발-로드맵-현황)

---

## 1. 프로젝트 개요

**Go2 Intelligence Framework**는 Unitree Go2 사족 보행 로봇을 위한 통합 자율 지능 시스템입니다.

| 계층 | 기술 |
|------|------|
| 시뮬레이터 | NVIDIA Isaac Sim 5.1.0 (Isaac Lab v2.3.0) |
| 보행 제어 | 강화학습(RSL-RL) 정책 (`policies/go2_policy.pt`) |
| 3D SLAM | RTAB-Map (RGB-D + 오도메트리) |
| 자율 내비게이션 | ROS 2 Nav2 스택 |
| 사용자 인터페이스 | PyQt5 기반 GUI + STT 자연어 명령 |
| 실 로봇 지원 | Unitree SDK2 / unitree_ros2, Intel RealSense D435i |

---

## 2. 디렉토리 구조

```text
go2_intelligence_framework/
│
├── assets/                         # 시뮬레이션용 USD 환경 파일 및 이미지
│   ├── go2_hero.png                # README 헤더 이미지
│   ├── simple_env.usd              # 기본 평지 환경 (단순 장애물)
│   └── slam_env.usd                # SLAM 전용 창고형 환경 (기둥·박스 배치)
│
├── config/                         # ROS 설정 파일
│   ├── go2_nav2_params.yaml        # Nav2 파라미터 (시뮬용)
│   ├── go2_nav2_params_real.yaml   # Nav2 파라미터 (실 로봇용)
│   └── go2_sim.rviz                # RViz2 시각화 설정 (SLAM·내비게이션 공용)
│
├── docs/                           # 설계 문서 & 개발 노트
│   ├── architecture.md             # ★ 이 파일 — 전체 구조 문서
│   ├── 00_archive_orb_slam3_task.md        # (아카이브) ORB-SLAM3 시도 기록
│   ├── 01_rtabmap_slam_plan.md             # RTAB-Map SLAM 설계
│   ├── 01_rtabmap_slam_presentation.md     # SLAM 발표 자료
│   ├── 02_policy_decision.md               # RL 정책 선택 근거
│   ├── 03_nav2_plan.md                     # Nav2 통합 계획
│   ├── 04_real_robot_deploy.md             # 실 로봇 배포 가이드
│   ├── 05_visual_odom_migration.md         # 시각적 오도메트리 마이그레이션
│   ├── 06_rviz_robot_model_plan.md         # RViz 로봇 모델 설정
│   ├── 07_gui_controller_plan.md           # GUI 컨트롤러 설계
│   ├── 08_semantic_navigation_direction.md # 시멘틱 내비게이션 방향
│   ├── 09_multifloor_stair_navigation_direction.md  # 다층/계단 내비게이션 방향
│   ├── 10_project_dependencies.md          # 의존성 목록 및 설치 기록
│   ├── 11_rtabmap_real_topics.md           # 실 로봇 RTAB-Map 토픽 정리
│   └── 12_rtabmap_real_review_actions.md   # 실 로봇 RTAB-Map 리뷰 액션
│
├── launch/                         # ROS 2 launch 파일
│   ├── go2_rtabmap.launch.py       # SLAM 스택 (시뮬용)
│   ├── go2_rtabmap_real.launch.py  # SLAM 스택 (실 로봇용)
│   ├── go2_navigation.launch.py    # 내비게이션 스택 (시뮬용)
│   └── go2_navigation_real.launch.py  # 내비게이션 스택 (실 로봇용)
│
├── maps/                           # RTAB-Map 데이터베이스 (gitignore, 런타임 생성)
│   ├── rtabmap.db                  # SLAM 모드로 생성된 현재 맵
│   ├── rtabmap_ground_truth.db     # 내비게이션용 확정 맵 (사용자가 수동 rename)
│   └── rtabmap_real.db             # 실 로봇용 맵
│
├── policies/                       # 사전 학습된 강화학습 정책
│   └── go2_policy.pt               # RSL-RL로 학습된 Go2 보행 정책 (PyTorch)
│
├── requirements.txt                # pip 의존성 (faster-whisper, sounddevice, PyYAML)
│
├── scripts/                        # 핵심 실행 스크립트
│   ├── go2_sim.py                  # ★ 메인 시뮬레이션 진입점
│   ├── my_slam_env.py              # Isaac Lab 환경 설정 (MySlamEnvCfg)
│   ├── cli_args.py                 # RSL-RL CLI 인자 파싱 유틸리티
│   ├── deploy_scene_mcp.py         # Isaac Sim MCP로 동적 장애물 배치
│   └── run_gui_controller.sh       # GUI 컨트롤러 실행 헬퍼 스크립트
│
└── src/                            # ROS 2 패키지 소스
    ├── go2_gui_controller/         # GUI 컨트롤러 패키지 (ament_python)
    │   ├── config/
    │   │   └── waypoints.yaml      # 웨이포인트 정의 파일
    │   ├── go2_gui_controller/     # Python 패키지 소스
    │   │   ├── __init__.py
    │   │   ├── camera_info_restamper.py  # CameraInfo 타임스탬프 재발행 노드
    │   │   ├── charts_panel.py     # pyqtgraph 기반 실시간 텔레메트리 차트 UI
    │   │   ├── commands.py         # 명령 데이터 클래스 (CommandType, ParsedCommand)
    │   │   ├── gui_app.py          # ★ GUI 메인 애플리케이션 (PyQt5 윈도우)
    │   │   ├── image_restamper.py  # Image 타임스탬프 재발행 노드
    │   │   ├── launch_manager.py   # SLAM/Nav2/RViz 프로세스 수명 관리
    │   │   ├── main.py             # ROS 2 노드 진입점
    │   │   ├── manual_control.py   # /cmd_vel Twist 발행 (수동 조작)
    │   │   ├── navigator_bridge.py # Nav2 NavigateToPose 액션 클라이언트
    │   │   ├── odom_restamper.py   # Odometry 타임스탬프 정규화 + 2D 평면화 노드
    │   │   ├── rgbd_odom_sync.py   # RGB-D + Odometry 시간 동기화 노드
    │   │   ├── rgbd_restamper.py   # RGB-D 스트림 타임스탬프 일괄 재발행 노드
    │   │   ├── state_bridge.py     # 로봇 위치 상태 추적 (TF + Odometry)
    │   │   ├── telemetry_bridge.py # 조인트 상태·속도·맵 토픽 수집 및 히스토리 관리
    │   │   ├── text_command_parser.py  # 텍스트/음성 명령 파싱 (한국어·영어)
    │   │   └── voice_command_listener.py  # faster-whisper STT 마이크 입력 처리
    │   ├── launch/
    │   │   └── go2_gui_controller.launch.py  # GUI 노드 launch 파일
    │   ├── resource/
    │   │   └── go2_gui_controller  # ament 리소스 마커
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py
    │
    └── go2_project_dependencies/   # ROS 2 의존성 번들 패키지 (ament_cmake)
        ├── CMakeLists.txt
        └── package.xml             # joint_state_publisher, nav2_bringup, rtabmap_ros 등 선언
```

---

## 3. 주요 모듈 설명

### 3.1 시뮬레이션 환경 (`scripts/`)

#### `go2_sim.py` — 메인 시뮬레이션 진입점

Isaac Lab 환경을 초기화하고 RSL-RL 정책을 실행하는 핵심 스크립트입니다.

주요 흐름:
1. Isaac Sim `AppLauncher`를 통해 시뮬레이터 기동
2. `MySlamEnvCfg`로 환경 구성 (`my_slam_env.py` 참조)
3. `RslRlVecEnvWrapper`로 환경 래핑 후 `OnPolicyRunner`가 `go2_policy.pt` 로드
4. Isaac Sim 내 OmniGraph를 통해 RGB 카메라·IMU·오도메트리를 ROS 2 토픽으로 브리지
5. `WasdKeyboard`로 W/A/S/D/Q/E 키 입력을 속도 명령으로 변환

#### `my_slam_env.py` — Isaac Lab 환경 설정

`UnitreeGo2RoughEnvCfg`를 상속하는 `MySlamEnvCfg` 클래스:

| 설정 항목 | 값 |
|-----------|-----|
| 지형 (`terrain`) | `slam_env.usd` (창고형 환경) |
| 카메라 (`front_cam`) | Intel RealSense D435 근사 (320×240, FOV 20.955mm) |
| IMU (`imu_sensor`) | 50Hz, base frame |
| 정책 obs 차원 | 45-dim (base_lin_vel, height_scan 제거) |
| 에피소드 길이 | 무한 (1.0e9 s) |
| 속도 명령 | heading_command=False (직접 ω_z 제어) |

#### `deploy_scene_mcp.py` — 동적 장애물 배치

Isaac Sim MCP(Model Communication Protocol) 소켓(포트 8766)으로 Python 스크립트를 원격 실행해 USD Stage에 PhysX 충돌 기둥을 자동 배치합니다. (장애물 회피 테스트용)

---

### 3.2 SLAM 스택 (`launch/go2_rtabmap*.launch.py`)

#### 시뮬용: `go2_rtabmap.launch.py`

| 노드 | 역할 |
|------|------|
| `robot_state_publisher` | URDF → TF (`base_link` 이하 관절 트리) |
| `joint_state_publisher` | `use_fake_joint_states=true` 시 기본 자세 발행 |
| `static_transform_publisher` × 2 | `base_link→camera_link`, `camera_link→camera_optical_frame` |
| `depthimage_to_laserscan` | depth image → `/scan` LaserScan 변환 (10행 평균, 0.2~5.0m) |
| `rtabmap` (SLAM 모드) | RGB-D + `/odom` + IMU → 3D 맵 생성, `maps/rtabmap.db` 기록 |
| `rtabmap` (Localization 모드) | `maps/rtabmap_ground_truth.db` 로드 → `map→odom` TF 발행 |

주요 RTAB-Map 파라미터 (시뮬):
- `Vis/EstimationType: 2` — 3D-3D 특징점 매칭 (depth 활용)
- `Vis/MinInliers: 15` — 저해상도 시뮬 텍스처 보완
- `Reg/Force3DoF: true` — 평지 실내 roll/pitch 영향 억제
- `Grid/CellSize: 0.05m` — 5cm 격자 점유 맵

#### 실 로봇용: `go2_rtabmap_real.launch.py`

시뮬 버전과 비교한 주요 차이점:

| 항목 | 시뮬 | 실 로봇 |
|------|------|---------|
| 오도메트리 소스 | Isaac Sim OmniGraph `/odom` | unitree_ros2 `/utlidar/robot_odom` |
| 카메라 입력 방식 | 개별 `rgb/depth/camera_info` | `rgbd_sync` → `/camera/rgbd_image` |
| 오도메트리 전처리 | 없음 | `odom_restamper` (타임스탬프 정규화 + 2D 평면화) |
| IMU 구독 | `subscribe_imu: true` | `subscribe_imu: false` (unitree_ros2 odom에 이미 융합) |
| 탐지 주기 | 0.5Hz (맵핑) / 2.0Hz (로컬라이제이션) | 2.5Hz (맵핑) / 2.0Hz (로컬라이제이션) |
| LaserScan 범위 | 0.2~5.0m | 0.3~4.0m (D435i 실용 범위) |

---

### 3.3 내비게이션 스택 (`launch/go2_navigation*.launch.py`)

`go2_navigation.launch.py` (시뮬) 및 `go2_navigation_real.launch.py` (실 로봇)은 각각:

1. `go2_rtabmap[_real].launch.py`를 **Localization 모드**로 포함
2. `nav2_bringup/navigation_launch.py`를 포함

```
RTAB-Map (Localization) ─► /map 토픽 + map→odom TF
                                   │
Nav2 스택 ─────────────────────────┘
  ├── bt_navigator       (행동 트리 기반 목표 실행)
  ├── planner_server     (전역 경로 계획, NavFn)
  ├── controller_server  (지역 제어, DWB)
  ├── behavior_server    (회복 행동: Spin, Wait, BackUp)
  └── costmap_2d         (전역/지역 비용 맵)
              │
              ▼
        /cmd_vel ──► Go2 보행 정책 (Isaac Sim) 또는 실 로봇
```

> Nav2에서 `map_server` / `amcl`은 사용하지 않습니다. RTAB-Map이 직접 `/map` 토픽과 `map→odom` TF를 발행합니다.

---

### 3.4 GUI 컨트롤러 (`src/go2_gui_controller/`)

PyQt5 기반 실시간 미션 제어 대시보드입니다. `run_gui_controller.sh`로 실행합니다.

#### 클래스 구조

```
gui_app.py (GuiApp / MainWindow)
├── StateBridge            ← /odom + /tf 구독, 로봇 위치·로컬라이제이션 상태 추적
├── TelemetryBridge        ← /joint_states, /cmd_vel, /scan, /map 수집 및 히스토리
├── NavigatorBridge        ← NavigateToPose 액션 클라이언트 (Nav2 연동)
├── ManualControlBridge    ← /cmd_vel Twist 발행 (수동 버튼 조작)
├── LaunchManager          ← SLAM / Navigation / RViz 프로세스 수명 관리
├── WaypointRegistry       ← waypoints.yaml 로드·저장·삭제·검색
├── TextCommandParser      ← 텍스트 명령 파싱 (한국어·영어)
├── VoiceCommandListener   ← faster-whisper STT (마이크 → 텍스트)
└── ChartsPanel            ← pyqtgraph 실시간 조인트·속도 차트
```

#### 주요 UI 기능

| 탭/패널 | 기능 |
|---------|------|
| **Manual Control** | 방향 버튼으로 직접 `/cmd_vel` 발행 |
| **Navigation** | 웨이포인트 목록 표시, 목표 전송, 취소 |
| **Waypoints** | 현재 위치 저장, 이름 변경, 삭제 |
| **Runtime** | SLAM/Nav2/RViz 프로세스 시작·중지 (시뮬/실 로봇 모드 전환) |
| **Telemetry** | 토픽 수신 상태, 로봇 위치/자세 텍스트 표시 |
| **Charts** | 조인트 위치·속도, 선속도·각속도 실시간 그래프 |
| **Voice/Text** | 텍스트 입력 또는 STT 마이크 입력 → 명령 파싱 → 자동 웨이포인트 이동 |

#### 타임스탬프 정규화 유틸리티 노드

| 파일 | 노드명 | 역할 |
|------|--------|------|
| `camera_info_restamper.py` | `camera_info_restamper` | `CameraInfo` 타임스탬프를 현재 ROS 시각으로 교체 |
| `image_restamper.py` | `image_restamper` | `Image` 타임스탬프 교체 |
| `rgbd_restamper.py` | `rgbd_restamper` | RGB + Depth + CameraInfo 3개 스트림을 동기화하여 일괄 타임스탬프 교체 |
| `odom_restamper.py` | `odom_restamper` | Odometry를 ROS 시각 기준으로 재발행 + 2D 평면화 (z/roll/pitch 제거), TF도 선택적 발행 |
| `rgbd_odom_sync.py` | `rgbd_odom_sync` | RGB-D 스트림과 Odometry를 시간 동기화하여 함께 재발행 |

---

### 3.5 ROS 패키지 의존성 (`src/go2_project_dependencies/`)

코드가 없는 ament_cmake 패키지입니다. `rosdep install`이 아래 ROS 패키지들을 자동 설치하도록 `package.xml`에 선언합니다.

- `joint_state_publisher`
- `nav2_bringup`
- `depthimage_to_laserscan`
- `robot_state_publisher`
- `rtabmap_ros`
- `rviz2`
- `xacro`

---

## 4. 실행 흐름 (시뮬레이션)

```
┌──────────────────────────────────────────────────────────────────┐
│  Terminal A: conda activate <env> && python scripts/go2_sim.py   │
│                                                                  │
│  Isaac Sim + Isaac Lab                                           │
│  ┌─────────────┐    RSL-RL 정책 추론                             │
│  │ MySlamEnvCfg│──► OnPolicyRunner ──► 관절 토크 출력           │
│  │ (slam_env   │                                                 │
│  │  .usd)      │    OmniGraph ROS2 Bridge                       │
│  │             │──► /camera/color/image_raw  (RGB 30Hz)         │
│  │             │──► /camera/depth/image_rect_raw (Depth 30Hz)   │
│  │             │──► /camera/camera_info                         │
│  │             │──► /imu/data                                   │
│  │             │──► /odom  (ground-truth odometry)              │
│  │             │──► /joint_states                               │
│  │             │──► /clock  (sim time)                          │
│  └─────────────┘                                                 │
└──────────────────────────────────────────────────────────────────┘
         │ ROS 2 DDS
         ▼
┌──────────────────────────────────────────────────────────────────┐
│  Terminal B: ros2 launch launch/go2_rtabmap.launch.py            │
│  (or go2_navigation.launch.py for Nav2)                         │
│                                                                  │
│  static_tf: base_link → camera_link (x=0.30, z=0.05)           │
│  static_tf: camera_link → camera_optical_frame (RPY -90°,0,-90°)│
│  robot_state_publisher: URDF → /tf (관절 체인)                   │
│  depthimage_to_laserscan: depth → /scan                         │
│  rtabmap: RGB-D + /odom + /imu → /map + map→odom TF             │
│  [Nav2 추가 시] bt_navigator / planner / controller              │
└──────────────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────────────────────────────┐
│  Terminal C: rviz2 -d config/go2_sim.rviz                        │
│  (또는 bash scripts/run_gui_controller.sh)                       │
└──────────────────────────────────────────────────────────────────┘
```

---

## 5. 실행 흐름 (실 로봇)

```
┌──────────────────────────────────────────────────────────────────┐
│  Unitree Go2 (온보드 PC)                                         │
│  ├── unitree_ros2: /utlidar/robot_odom, /joint_states           │
│  └── realsense2_camera: /camera/color/image_raw                 │
│                         /camera/aligned_depth_to_color/image_raw│
│                         /camera/color/camera_info               │
└──────────────────────────────────────────────────────────────────┘
         │ Wi-Fi / Ethernet (ROS 2 DDS)
         ▼
┌──────────────────────────────────────────────────────────────────┐
│  개발 PC                                                         │
│  bash scripts/run_gui_controller.sh                             │
│  → LaunchManager가 아래 스택 자동 실행:                          │
│                                                                  │
│  go2_rtabmap_real.launch.py:                                    │
│    odom_restamper: /utlidar/robot_odom → /utlidar/robot_odom_   │
│                   _restamped  (타임스탬프 정규화 + 2D 평면화)   │
│    rgbd_sync: RGB + Depth + CameraInfo → /camera/rgbd_image     │
│    static_tf: base_link → camera_link (x=0.33, z=0.09)         │
│    static_tf: camera_link → camera_color_optical_frame          │
│    depthimage_to_laserscan: aligned_depth → /scan               │
│    rtabmap: /camera/rgbd_image + odom TF → /map + map→odom TF   │
│                                                                  │
│  go2_navigation_real.launch.py (추가 시):                       │
│    Nav2 스택: /cmd_vel 발행                                      │
│                                                                  │
│  GUI: 웨이포인트 설정 → NavigateToPose 액션 → /cmd_vel → Go2   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 6. ROS 토픽 맵

### 시뮬레이션에서 발행되는 토픽 (Isaac Sim → ROS 2)

| 토픽 | 메시지 타입 | 발행 주체 | 설명 |
|------|-------------|-----------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Isaac Sim OmniGraph | RGB 카메라 영상 (320×240, 30Hz) |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Isaac Sim OmniGraph | Depth 카메라 영상 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Isaac Sim OmniGraph | 카메라 내부 파라미터 |
| `/imu/data` | `sensor_msgs/Imu` | Isaac Sim OmniGraph | IMU (50Hz, base frame) |
| `/odom` | `nav_msgs/Odometry` | Isaac Sim OmniGraph | Ground-truth 오도메트리 |
| `/joint_states` | `sensor_msgs/JointState` | Isaac Sim OmniGraph | 12개 관절 위치·속도 |
| `/clock` | `rosgraph_msgs/Clock` | Isaac Sim | 시뮬레이션 시각 |

### SLAM/Nav2가 발행하는 토픽

| 토픽 | 메시지 타입 | 발행 주체 | 설명 |
|------|-------------|-----------|------|
| `/scan` | `sensor_msgs/LaserScan` | `depthimage_to_laserscan` | 2D 레이저 스캔 |
| `/map` | `nav_msgs/OccupancyGrid` | `rtabmap` | 2D 점유 격자 맵 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller / GUI | 속도 명령 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 bt_navigator | 목표 위치 내비게이션 액션 |

### 실 로봇 추가 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | unitree_ros2 원본 오도메트리 |
| `/utlidar/robot_odom_restamped` | `nav_msgs/Odometry` | odom_restamper 처리 후 오도메트리 |
| `/camera/rgbd_image` | `rtabmap_msgs/RGBDImage` | rgbd_sync 동기화 스트림 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense aligned depth |

---

## 7. TF 프레임 트리

### 시뮬레이션 TF 트리

```
map
 └── odom  (RTAB-Map 발행 — Localization 모드 시)
      └── base_link  (robot_state_publisher + Isaac Sim)
           ├── [12개 관절 프레임]  (robot_state_publisher)
           ├── camera_link  (static_transform_publisher: x=0.30, z=0.05)
           │    └── camera_optical_frame  (static: RPY -90°, 0, -90°)
           └── base_footprint  (robot_state_publisher)
```

### 실 로봇 TF 트리

```
map
 └── odom  (RTAB-Map 발행)
      └── base_link  (unitree_ros2 또는 odom_restamper TF)
           ├── camera_link  (static: x=0.33, z=0.09)
           │    └── camera_color_optical_frame  (static: RPY -90°, 0, -90°)
           └── base_footprint
```

---

## 8. 주요 클래스 & 파일 참조

| 클래스 / 함수 | 파일 | 역할 |
|--------------|------|------|
| `MySlamEnvCfg` | `scripts/my_slam_env.py` | Isaac Lab 환경 구성 (지형·센서·정책 obs) |
| `WasdKeyboard` | `scripts/go2_sim.py` | WASDQE 키 → 속도 명령 변환 |
| `StateBridge` | `src/.../state_bridge.py` | TF 버퍼 + Odometry로 로봇 위치 추적, 로컬라이제이션 감지 |
| `RobotState` | `src/.../state_bridge.py` | x, y, yaw, frame_id, nav_status 등 상태 데이터 클래스 |
| `NavigatorBridge` | `src/.../navigator_bridge.py` | Nav2 액션 클라이언트, 웨이포인트/상대 이동 전송 |
| `TelemetryBridge` | `src/.../telemetry_bridge.py` | 토픽 수신 통계·히스토리 버퍼 관리 (최대 20초) |
| `WaypointRegistry` | `src/.../waypoint_registry.py` | YAML 기반 웨이포인트 CRUD (별칭 포함) |
| `TextCommandParser` | `src/.../text_command_parser.py` | 텍스트 명령 → `ParsedCommand` (이동/회전/웨이포인트/정지) |
| `VoiceCommandListener` | `src/.../voice_command_listener.py` | sounddevice 녹음 → faster-whisper STT → 텍스트 |
| `LaunchManager` | `src/.../launch_manager.py` | QProcess 기반 SLAM/Nav2/RViz 프로세스 관리 |
| `OdomRestamper` | `src/.../odom_restamper.py` | ROS 시각 기준 타임스탬프 재발행 + 2D 평면화 |
| `RgbdOdomSync` | `src/.../rgbd_odom_sync.py` | RGB-D와 Odometry를 시간 동기화하여 함께 재발행 |
| `ChartsPanel` | `src/.../charts_panel.py` | pyqtgraph 기반 실시간 센서 차트 UI |

---

## 9. 의존성 요약

### 시스템 / 수동 설치

| 소프트웨어 | 버전 | 비고 |
|-----------|------|------|
| Ubuntu | 22.04 LTS | |
| ROS 2 | Humble Hawksbill | |
| NVIDIA Isaac Sim | 5.1.0 | GPU 필수 |
| NVIDIA Isaac Lab | v2.3.0 | Isaac Sim 확장 |
| `go2_description` | - | URDF, 별도 워크스페이스 |

### pip (`requirements.txt`)

| 패키지 | 용도 |
|--------|------|
| `faster-whisper` | STT 음성 인식 (Whisper 모델) |
| `sounddevice` | 마이크 오디오 캡처 |
| `PyYAML` | 웨이포인트 YAML 파싱 |

### rosdep (`src/` 패키지 기반 자동 설치)

**go2_gui_controller** (Python):
`action_msgs`, `geometry_msgs`, `nav_msgs`, `nav2_msgs`, `nav2_simple_commander`,
`portaudio19-dev`, `python3-numpy`, `python3-pyqtgraph`, `python_qt_binding`,
`rclpy`, `sensor_msgs`, `tf2_ros`, `ament_index_python`

**go2_project_dependencies** (빌드 없음, 선언만):
`joint_state_publisher`, `nav2_bringup`, `depthimage_to_laserscan`,
`robot_state_publisher`, `rtabmap_ros`, `rviz2`, `xacro`

---

## 10. 개발 로드맵 현황

| 단계 | 내용 | 상태 |
|------|------|------|
| Phase 1 | 3D SLAM — RTAB-Map + Isaac Sim | ✅ 완료 |
| Phase 2 | 자율 내비게이션 — Nav2 통합 | ✅ 완료 |
| Phase 3 | 지능형 GUI & 미션 제어 대시보드 | ✅ 완료 |
| Phase 4 | 실 로봇 Sim2Real 배포 (STT 명령 포함) | ✅ 완료 |
| Phase 5 | LLM 기반 고급 추론 & 장면 이해 | 🔲 계획 중 |

---

*최종 업데이트: 2026-03-30*
