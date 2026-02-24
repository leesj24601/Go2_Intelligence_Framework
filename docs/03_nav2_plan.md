# Implementation Plan: Go2 Nav2 Navigation in Isaac Sim → Real Robot

## Goal
Isaac Sim 환경에서 RTAB-Map으로 생성한 맵(또는 실시간 SLAM 중인 맵)과 Odometry/TF를 기반으로,
ROS2 Navigation2 (Nav2) 스택을 연동하여 Go2 로봇이 목표 지점(Goal Pose)까지
스스로 장애물을 회피하며 이동(자율주행)하도록 구현한다.
최종 목표는 실제 Go2 로봇에서도 동일하게 동작하는 것.

---

## 전제 조건 (현재 상태)

| 항목 | 상태 | 비고 |
|------|------|------|
| Nav2 패키지 | ✅ 설치됨 | nav2_bringup, nav2_mppi_controller 등 |
| depthimage_to_laserscan | ✅ 설치됨 | Phase 2에서 사용 |
| RTAB-Map | ✅ 구성됨 | `launch/go2_rtabmap.launch.py` |
| TF 트리 | ✅ 구성됨 | `odom → base_link → camera_link → camera_optical_frame` |
| /odom 토픽 | ✅ 발행 중 | OmniGraph IsaacComputeOdometry |
| unitree_rl_lab 정책 | 🔄 훈련 중 | 시뮬은 기존 정책으로 먼저 검증 |

## TF 트리 (완성 형태)

```
map  ←─ RTAB-Map이 발행 (map→odom TF)
 └── odom  ←─ OmniGraph 발행
      └── base_link
           └── camera_link
                └── camera_optical_frame
```

> ⚠️ AMCL 불필요: RTAB-Map이 `map → odom` TF를 직접 발행하므로 AMCL 대체

## 아키텍처 흐름

```
[Isaac Sim / Real Robot]
  ├─ Depth Image → depthimage_to_laserscan → /scan
  ├─ /odom, /tf (map→odom→base_link)
  └─ /cmd_vel 수신 → RL Policy → 관절 제어
          ↑
    [Nav2 Stack]
      ├─ RTAB-Map: 맵 생성 + map→odom TF 발행
      ├─ Costmap2D: /scan 기반 장애물 레이어
      ├─ NavFn (Global Planner): A* 경로 생성
      ├─ MPPI Controller (Local Planner): cmd_vel 계산
      └─ BT Navigator: 상태 관리
```

---

## Phase 0: 사전 준비 (환경 정의) ✅ 완료

### Go2 로봇 파라미터 정의 (확정)
```
footprint: [[-0.35, -0.20], [-0.35, 0.20], [0.35, 0.20], [0.35, -0.20]]  # 700x400mm
max_vel_x: 1.0 m/s
max_vel_y: 0.4 m/s  (사족보행 → 횡이동 가능)
max_vel_theta: 1.0 rad/s
```

### footprint 결정 근거
- URDF (`go2_ws/src/go2_description/urdf/go2_description.urdf`) 분석:
  - base_link collision box: 376×94mm (단순화 모델, 실제보다 작음)
  - hip joint 스팬(x): ±193mm → 전후 합계 387mm (head 미포함)
  - head 최전방(x): base_link 중심에서 +340mm (Head_lower + 반지름 47mm)
  - 후방 RL hip ~ head 끝 전체 x: 193 + 340 = **533mm**
  - 다리 최대 폭(y): hip(±47mm) + thigh(±96mm) + foot radius(22mm) = **±164mm → 328mm**
- 공식 스펙 (Unitree): 서있는 자세 700×310mm
  - 700mm: 전체 로봇 길이 ✅
  - 310mm: **몸통 폭만** 측정한 값. 다리 포함 실제 폭은 328mm로 더 넓음
- **채택: 700×400mm** — 공식 스펙 x(700mm) + 다리 폭(328mm)에 안전 마진 추가(400mm)
- yaml 숫자만 바꾸면 Nav2 재시작 시 즉시 반영 가능

### 최대 속도 결정 근거
- IsaacLab 훈련 범위 (`velocity_env_cfg.py`): lin_vel_x/y=±1.0, ang_vel_z=±1.0
- **1.5 m/s는 훈련 범위 초과** → 정책 불안정 위험
- 논문 실측값 (arxiv 2504.17880, Go2 Edu): x=1.0, y=0.5, theta=0.8
- unitree_rl_lab `limit_ranges` (실제 훈련 최대 범위): x=±1.0, y=±0.4, ang_z=±1.0
- **채택: x=1.0, y=0.4, theta=1.0** — unitree_rl_lab 훈련 범위 기준 (논문 y=0.5는 범위 초과)

> ⚠️ **재확인 필요**: 현재 속도는 기본 Isaac Lab 정책 기준. unitree_rl_lab 정책으로 교체 시
> 해당 정책의 훈련 범위(`vel_range` 또는 `command_ranges`)를 확인하여 이 값을 재조정해야 함.
> Phase 6 실로봇 배포 전 반드시 재검토.

### Controller 선택: MPPI (확정)
- DWB: 바퀴형 로봇(Differential) 전용 → Go2에 부적합
- RPP (Regulated Pure Pursuit): 단방향 로봇에 적합 → Go2에 제한적
- **MPPI**: 모델 예측 기반, omnidirectional 지원, 복잡한 환경에 강인 → **Go2 최적**

### 작업
- [x] Go2 footprint 정확한 치수 측정/확인
- [x] 시뮬 기준 최대 속도 확인 (훈련 범위 기반)

---

## Phase 1: cmd_vel → Isaac Sim 연동 ⭐ ✅ 완료

> Nav2 구성 전에 cmd_vel이 로봇을 실제로 움직이게 하는 것이 최우선

### 구현 내용 (완료)
```
/cmd_vel (Twist)
  └─ CmdVelNode (Isaac Sim 번들 rclpy, 별도 스레드 spin_once 루프)
       └─ linear.x → cmd_term.vel_command_b[0, 0]
          linear.y → cmd_term.vel_command_b[0, 1]
          angular.z → cmd_term.vel_command_b[0, 2]
```

### 우선순위 설계 (구현됨)
```
/cmd_vel 수신 중 → cmd_vel 우선 적용 (Nav2 또는 방향키 테스트)
/cmd_vel 타임아웃(0.5s) → 자동 정지 → WASD 폴백
```

### rclpy 환경
- conda lab (Python 3.11)에서 Isaac Sim 번들 rclpy 사용
- `/opt/ros/` 경로 차단 → 번들 경로를 sys.path 최상단 등록 (go2_sim.py 최상단)
- `spin_once(timeout_sec=0.01)` 루프로 GIL 점유 최소화

### 테스트 방법
- ArrowKeyboard (↑↓←→) → /cmd_vel 발행 → 수신 → 로봇 이동 확인 ✅
- 나중에 Nav2로 교체 시: 방향키 코드 제거, Nav2가 /cmd_vel 발행하면 동일 동작

### 작업
- [x] `go2_sim.py`에 rclpy 초기화 및 `/cmd_vel` subscriber 추가
- [x] 별도 스레드에서 `rclpy.spin_once()` 루프 실행 (GIL 최소화)
- [x] 수신한 Twist를 velocity buffer에 매핑
- [x] 방향키로 `/cmd_vel` 발행하여 실제 이동 확인
- [x] 타임아웃 안전장치 구현 (0.5초)

---

## Phase 2: 센서 변환 (Depth → LaserScan) ✅ 완료

### 목표
Depth 이미지를 Nav2 Costmap이 사용할 2D LaserScan으로 변환

### 구현 내용

`launch/go2_rtabmap.launch.py`에 `depthimage_to_laserscan` 노드 추가:

```python
depthimage_to_laserscan = Node(
    package="depthimage_to_laserscan",
    executable="depthimage_to_laserscan_node",
    parameters=[{
        "scan_height": 10,       # 중앙 10행 평균 → 노이즈 감소 (1행보다 안정적)
        "scan_time": 0.1,
        "range_min": 0.2,
        "range_max": 5.0,
        "output_frame": "camera_link",
    }],
    remappings=[
        ("depth", "/camera/depth/image_rect_raw"),        # ⚠️ "image" 아님
        ("depth_camera_info", "/camera/camera_info"),     # ⚠️ "camera_info" 아님
        ("scan", "/scan"),
    ],
)
```

> ⚠️ **주의**: `depthimage_to_laserscan` 노드의 내부 토픽명은 `image`/`camera_info`가 아니라
> `depth`/`depth_camera_info`임. `ros2 node info`로 확인 필수.

### 실측 결과
- 실제 발행 주파수: **~4.4Hz** (설정 10Hz보다 낮음 — depth 카메라 발행 속도에 종속)
- scan 위치: 카메라 높이(ground + ~0.33m)에서 수평 스캔
- 장애물 감지: 벽/박스 표면에 정확히 찍힘 ✅

> ℹ️ 대안: PointCloud2를 costmap의 obstacle_layer에 직접 연결 가능
> (더 많은 정보 활용, 계산량 증가)

### 작업
- [x] `depthimage_to_laserscan` 노드 launch 설정
- [x] scan 높이(지면 기준 카메라 높이) 파라미터 조정 (`scan_height: 10`)
- [x] RViz2에서 `/scan` 토픽 확인 (지면 평행, 장애물 감지 확인)

---

## Phase 3: Nav2 파라미터 설정 ✅ 완료

### 파일 생성: `config/go2_nav2_params.yaml`

#### 핵심 파라미터 구조 (실제 적용값)
```yaml
controller_server:
  controller_frequency: 10.0
  FollowPath:
    plugin: "nav2_mppi_controller::MPPIController"
    motion_model: "Omni"    # Go2: 사족보행 omnidirectional
    time_steps: 56
    model_dt: 0.05          # horizon: 56 * 0.05 = 2.8s
    vx_max: 1.0             # unitree_rl_lab limit_ranges 기준
    vx_min: -0.5
    vy_max: 0.4             # unitree_rl_lab limit_ranges 기준
    wz_max: 1.0             # unitree_rl_lab limit_ranges 기준

local_costmap:
  footprint: "[[-0.35,-0.20],[-0.35,0.20],[0.35,0.20],[0.35,-0.20]]"
  plugins: ["voxel_layer", "inflation_layer"]
  voxel_layer:
    observation_sources: scan  # /scan (depthimage_to_laserscan)
  inflation_layer:
    inflation_radius: 0.55    # footprint 최대반경 ~0.40 + 여유 0.15

global_costmap:
  footprint: "[[-0.35,-0.20],[-0.35,0.20],[0.35,0.20],[0.35,-0.20]]"
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  inflation_layer:
    inflation_radius: 0.55

planner_server:
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    use_astar: true

velocity_smoother:
  max_velocity: [1.0, 0.4, 1.0]   # [x, y, theta]
```

> ℹ️ `velocity_smoother`, `collision_monitor`, `behavior_server` (spin/backup) 모두 포함됨

### 작업
- [x] `config/go2_nav2_params.yaml` 작성
- [x] MPPI controller 파라미터 튜닝 (motion_model=Omni, horizon=2.8s)
- [x] Global/Local costmap 인플레이션 반경 설정 (0.55m)
- [x] Footprint 정확히 설정 (700×400mm)

---

## Phase 4: Launch 파일 통합

### 파일 생성: `launch/go2_navigation.launch.py`

#### 실행 노드 구성
```
go2_navigation.launch.py
  ├─ go2_rtabmap.launch.py  (SLAM 모드 또는 Localization 모드)
  ├─ depthimage_to_laserscan
  └─ nav2_bringup (nav2_params.yaml 적용)
```

#### 두 가지 모드 분기
```python
# 파라미터로 모드 선택
slam_mode = LaunchConfiguration('slam', default='true')

# SLAM 모드: 맵 생성하면서 동시에 주행
# Localization 모드: 기존 맵 로드 후 주행
```

### 작업
- [ ] `launch/go2_navigation.launch.py` 작성
- [ ] SLAM 모드 / Localization 모드 런치 파라미터 분기
- [ ] 전체 노드 한 번에 실행 확인

---

## Phase 5: 시뮬 테스트 및 튜닝

### 테스트 순서
1. Isaac Sim 실행 (`go2_sim.py`)
2. Nav2 전체 실행 (`go2_navigation.launch.py`)
3. RViz2 실행 (`go2_sim.rviz` + Nav2 패널 추가)
4. RViz2에서 `2D Goal Pose` 클릭 → 이동 확인

### 검증 항목
- [ ] map → odom TF 정상 발행 (RTAB-Map)
- [ ] /scan 데이터 costmap 반영 확인
- [ ] Goal Pose 수신 후 경로 계획 확인
- [ ] 장애물 앞 정지 및 우회 확인
- [ ] cmd_vel 값이 로봇 속도 제한 내에 있는지 확인

### 튜닝 포인트
- costmap 인플레이션 반경: **0.55m 적용** (footprint 최대반경 ~0.40 + 여유 0.15)
- MPPI horizon: **2.8s 적용** (time_steps=56, model_dt=0.05) — 너무 짧으면 진동, 너무 길면 느림
- recovery behavior: spin, backup **활성화됨** (behavior_server에 포함)

---

## Phase 6: 실로봇 배포 (unitree_rl_lab 정책 완성 후)

> unitree_rl_lab 훈련 완료 후 진행

### 시뮬 → 실로봇 변경 사항

| 항목 | 시뮬 | 실로봇 |
|------|------|--------|
| 센서 | OmniGraph 가상 카메라 | RealSense D435i 실제 |
| /odom | OmniGraph 계산 | unitree_sdk2 odometry |
| cmd_vel 수신 | go2_sim.py | unitree_rl_lab deploy 코드 |
| 클록 | /clock (sim time) | 시스템 시간 |

### 추가 작업
- [ ] unitree_rl_lab `deploy/robots/go2/` 코드에 `/cmd_vel` 수신 구현
- [ ] 실 RealSense 드라이버 연동 확인
- [ ] Nav2 파라미터 실로봇용 속도 제한 재조정 (실환경 안전 마진)
- [ ] 실로봇 테스트 환경에서 안전 구역 설정

---

## 작업 목록 (전체)

### Phase 0 ✅ 완료
- [x] Go2 footprint 확정: 700×400mm (±0.35, ±0.20)
- [x] 최대 속도 확정: x=1.0, y=0.4, theta=1.0 rad/s (unitree_rl_lab limit_ranges 기반)

### Phase 1 ⭐ ✅ 완료
- [x] `go2_sim.py`에 `/cmd_vel` subscriber 추가 (별도 스레드)
- [x] Twist → velocity buffer 매핑
- [x] 방향키로 동작 검증 (실제 이동 확인)
- [x] 타임아웃 안전장치 구현 (0.5초)

### Phase 2 ✅ 완료
- [x] `depthimage_to_laserscan` launch 설정 (`go2_rtabmap.launch.py`에 통합)
- [x] RViz2에서 `/scan` 검증 (4.4Hz, 장애물 감지 확인)

### Phase 3 ✅ 완료
- [x] `config/go2_nav2_params.yaml` 작성 (MPPI Omni, horizon=2.8s)
- [x] MPPI / costmap 파라미터 튜닝 (inflation_radius=0.55, velocity_smoother 포함)

### Phase 4
- [ ] `launch/go2_navigation.launch.py` 작성
- [ ] SLAM/Localization 모드 분기

### Phase 5
- [ ] 시뮬 전체 통합 테스트
- [ ] RViz2 Goal Pose 동작 확인
- [ ] 장애물 회피 확인 및 튜닝

### Phase 6
- [ ] 실로봇 cmd_vel 연동 (unitree_rl_lab deploy 코드)
- [ ] 실 RealSense 연동
- [ ] 실로봇 통합 테스트

---

## 트러블슈팅

### [Phase 1] conda lab 환경에서 rclpy import 실패

#### 원인

rclpy는 Python에서 ROS2와 통신하기 위한 Python 바인딩 라이브러리다.
내부적으로 C로 작성된 extension (`.so` 파일)을 포함하며, 이 파일은 **컴파일 시점의 Python 버전에 고정**된다.

```
_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so  ← Python 3.10 전용
_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so  ← Python 3.11 전용
```

문제는 `go2_sim.py`가 실행되는 conda `lab` 환경에는 **rclpy가 설치되어 있지 않다**.
`import rclpy`를 만나면 Python은 `sys.path`를 순서대로 탐색해 rclpy를 찾는다.
이때 `source /opt/ros/humble/setup.bash`로 인해 `/opt/ros/humble/local/lib/python3.10/dist-packages`가 `PYTHONPATH`에 등록되어 있으므로, Python은 이 경로에서 rclpy(3.10용)를 발견하고 로드를 시도한다.

그러나 conda `lab` 환경은 **Python 3.11**이므로, 3.10용 `.so`를 로드하는 순간 ABI(바이너리 인터페이스) 불일치로 실패한다.

```
ImportError: /opt/ros/.../rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so:
  cannot open shared object file (wrong Python version)
```

> rclpy는 pip으로 설치할 수 없다. ROS2 생태계 전체(`rcl`, `rmw`, `fastdds` 등 C 라이브러리)에 의존하므로 apt/rosdep 기반으로 배포된다. conda lab에 별도로 설치하는 것도 불가능하다.

#### 해결

Isaac Sim은 자체적으로 **Python 3.11용 rclpy**를 번들로 포함하고 있다.

```
번들 경로:
/home/cvr/anaconda3/envs/lab/lib/python3.11/site-packages/
  isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy/
    _rclpy_pybind11.cpython-311-x86_64-linux-gnu.so  ← 3.11용
```

`go2_sim.py` **최상단** (`import` 이전)에서 sys.path를 조작한다.
타이밍이 핵심으로, 어떤 모듈 import 이후에 조작하면 이미 캐싱된 모듈이 남아 해결이 안 된다.

```python
import sys

_ISAAC_ROS2_PATH = (
    "/home/cvr/anaconda3/envs/lab/lib/python3.11/site-packages"
    "/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy"
)
# 시스템 Python 3.10 경로 제거
sys.path = [p for p in sys.path if not p.startswith("/opt/ros/")]
# 번들 경로를 맨 앞에 삽입
if _ISAAC_ROS2_PATH not in sys.path:
    sys.path.insert(0, _ISAAC_ROS2_PATH)
```

#### rclpy를 사용하는 3가지 환경 정리

| 환경 | Python | rclpy | 용도 |
|------|--------|-------|------|
| 시스템 | 3.10 | `/opt/ros/humble/` | ros2 CLI, rviz2, RTAB-Map 등 |
| conda lab | 3.11 | 없음 | Isaac Lab/Sim 실행 전용 |
| Isaac Sim 번들 | 3.11 | `isaacsim.ros2.bridge/humble/rclpy/` | go2_sim.py 내부 rclpy 사용 |

---

### [Phase 1] rclpy.spin() 사용 시 시뮬레이션 렉 발생

#### 원인

`rclpy.spin(node)`은 블로킹 호출이며 Python GIL을 장시간 점유한다.
Isaac Sim 메인 루프와 GIL을 공유하므로, spin 스레드가 GIL을 잡고 있는 동안 시뮬레이션 루프가 멈춰 렉이 발생한다.

#### 해결

`spin_once(timeout_sec=0.01)` 루프로 교체한다.
짧은 timeout마다 GIL을 해제하여 메인 루프와 시간을 나눠 쓴다.

```python
# ❌ 잘못된 방법: GIL 장시간 점유
threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()

# ✅ 올바른 방법: GIL 주기적 해제
def _spin_loop():
    while rclpy.ok():
        rclpy.spin_once(self._node, timeout_sec=0.01)

threading.Thread(target=_spin_loop, daemon=True).start()
```
