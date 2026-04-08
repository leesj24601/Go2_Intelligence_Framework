---
tags: [project, deploy, real-robot]
status: in-progress
project: go2_intelligence_framework
type: implementation-plan
created: 2026-02-24
---

# Phase 6: 실로봇 배포

> 사전 조건: unitree_rl_lab 훈련 완료 + 시뮬 검증 완료 ([[03_nav2_plan]] Phase 0~5)

---

## 전체 구조

시뮬의 `go2_sim.py` 역할을 실로봇에서는 `unitree_rl_lab deploy`가 대신한다.
두 프로세스가 반드시 동시에 실행되어야 한다.

```
[시뮬]                              [실로봇]
go2_sim.py (Isaac Sim)          =   unitree_rl_lab deploy (C++)
  └─ RL policy → 관절 제어            └─ RL policy → 관절 제어 (DDS)

Nav2 + RTAB-Map (동일)          =   Nav2 + RTAB-Map (동일)
  └─ /cmd_vel 발행                    └─ /cmd_vel 발행
```

---

## 실행 방법

> ⚠️ `go2_ctrl` 실행 터미널에 `source /opt/ros/humble/setup.bash` 되어 있으면
> CycloneDDS iceoryx 버전 충돌로 크래시 발생. **반드시 ROS2 source 없는 터미널** 사용.

### MuJoCo sim2sim (지형 환경)

> 빌드 위치: `/home/cvr/Desktop/sj/unitree_mujoco/simulate/build/`

```bash
# 터미널 1: MuJoCo 시뮬레이터
cd /home/cvr/Desktop/sj/unitree_mujoco/simulate/build
./unitree_mujoco -r go2 -s scene_terrain.xml   # 지형 환경
# ./unitree_mujoco -r go2 -s scene.xml         # 평지 환경

# 터미널 2: RL 컨트롤러 (ROS2 source 없는 상태)
cd /home/cvr/Desktop/sj/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --network lo
```

**RL 정책 활성화 순서:**
```
1. MuJoCo 창 클릭 → 스페이스바 (시뮬레이션 시작)
2. LT + A       → FixStand 모드 (일어서기)
3. ≡ (Start)    → Velocity 모드 (RL 정책 활성화)
```

`config.yaml`에서 기본 씬 변경:
```yaml
# /home/cvr/Desktop/sj/unitree_mujoco/simulate/config.yaml
robot_scene: "scene_terrain.xml"   # 매번 -s 인자 없이 지형 환경으로 기본 실행
```

---

### 실로봇 deploy

```bash
# 터미널 1: RL 컨트롤러 (ROS2 source 없는 상태)
cd /home/cvr/Desktop/sj/unitree_rl_lab/deploy/robots/go2/build
./go2_ctrl --network eno1

# 터미널 2: Nav2 + RTAB-Map (ROS2 터미널)
ros2 launch go2_navigation_real.launch.py
```

**실행 전 체크 (로봇 SSH 후):**
```bash
sudo ps aux | grep -E "sport|loco|motion|go2|ctrl" | grep -v grep
# 출력 없으면 실행 가능. go2_ctrl이 시작 시 LowCmd 채널 점유 여부 자동 체크.
```

> ℹ️ `The other process is using the lowcmd channel` 경고가 떠도 실제 제어에는 문제없음.
> (`exit(0)`이 주석 처리되어 있어 경고 후 계속 진행)

---

### 통신 레이어
```
[deploy 코드]   DDS ↔ 로봇 하드웨어 (관절/IMU/조이스틱)
                + rclcpp 추가 예정 → /cmd_vel 수신

[ROS2 브릿지]   unitree_ros2 패키지
                DDS LowState → /odom, /tf 발행 → Nav2 수신
```

> ℹ️ unitree_ros2 브릿지는 Nav2가 로봇 위치를 알기 위해 필요.
> 설치/실행 여부 및 토픽명은 로봇 연결 후 확인 필요.

---

## RL 정책 obs 구조 및 cmd_vel 연동 원리

RL 정책 obs 벡터에는 **velocity_commands (vx, vy, omega)** 가 포함된다.
현재는 조이스틱이 이 값을 채워주고, Nav2는 이 자리를 `/cmd_vel`로 대체한다.

```cpp
// deploy/include/isaaclab/envs/mdp/observations/observations.h
REGISTER_OBSERVATION(velocity_commands)
{
    auto & joystick = env->robot->data.joystick;
    obs[0] = clamp(joystick->ly(), ...);    // vx  ← Nav2 cmd_vel.linear.x 로 대체
    obs[1] = clamp(-joystick->lx(), ...);   // vy  ← Nav2 cmd_vel.linear.y 로 대체
    obs[2] = clamp(-joystick->rx(), ...);   // omega ← Nav2 cmd_vel.angular.z 로 대체
    return obs;
}
```

**수정 방향**: cmd_vel 수신 중이면 cmd_vel 우선, 없으면 조이스틱 폴백
```cpp
if (cmd_vel_available && !timeout) {
    obs[0] = clamp(cmd_vel_vx,    -0.5, 1.0);
    obs[1] = clamp(cmd_vel_vy,    -0.4, 0.4);
    obs[2] = clamp(cmd_vel_omega, -1.0, 1.0);
} else {
    obs[0] = clamp(joystick->ly(), ...);   // 폴백
    obs[1] = clamp(-joystick->lx(), ...);
    obs[2] = clamp(-joystick->rx(), ...);
}
```

---

## 시뮬 → 실로봇 변경 사항

| 항목 | 시뮬 | 실로봇 |
|------|------|--------|
| 관절 제어 | go2_sim.py (Isaac Sim) | unitree_rl_lab deploy (C++, DDS) |
| 카메라 | OmniGraph 가상 카메라 | RealSense D435i 드라이버 |
| /odom, /tf | OmniGraph 계산 → ROS2 (ground truth) | unitree_ros2 LiDAR+IMU 퓨전 → ROS2 |
| /cmd_vel 수신 | go2_sim.py CmdVelNode (Python) | deploy observations.h 수정 (C++) |
| 클록 | /clock (sim time) | 시스템 시간 |
| use_sim_time | true | false |
| RViz | go2_sim.rviz | go2_sim.rviz (동일) |

---

## Phase A: cmd_vel 수신 구현 (unitree_rl_lab deploy)

> 수정 파일: `deploy/include/isaaclab/envs/mdp/observations/observations.h`
> 현재 deploy 코드에 ROS2 없음 → rclcpp 추가 필요

- [ ] `CMakeLists.txt`에 `rclcpp`, `geometry_msgs` 의존성 추가
- [ ] 글로벌 cmd_vel 수신 스레드 추가 (`rclcpp::init` + `spin_some` 루프, DDS 루프와 병행)
- [ ] `velocity_commands` observation 수정
  - cmd_vel 수신 시: cmd_vel 값으로 obs 채움 (clamp 적용)
  - 0.5초 타임아웃 시: 조이스틱 폴백
- [ ] 빌드 확인

---

## Phase B: 로봇 연결 후 확인 항목

> ⚠️ 아래 항목은 로봇 연결 전에는 알 수 없음. 연결 즉시 확인.

```bash
ros2 topic list         # /odom 토픽명, /tf 발행 여부 확인
ros2 run tf2_ros tf2_monitor    # odom → base_link TF 발행 주체 확인
```

- [ ] unitree_ros2 브릿지 설치/실행 여부 확인
- [x] LiDAR+IMU raw odom 토픽 확인: `/utlidar/robot_odom`
  → `go2_rtabmap_real.launch.py` raw 입력 기준과 일치
  → Nav2 소비 토픽은 raw timestamp(2024년 문제)를 피하기 위해 `/utlidar/robot_odom_restamped` 사용
- [x] `odom → base_link` TF는 upstream publisher가 직접 발행 중
  → `odom_restamper_publish_tf` 기본값은 `false` 유지
- [ ] RealSense 드라이버 토픽명 확인 → `go2_rtabmap_real.launch.py` remapping과 일치 여부

---

## Phase C: Launch 파일 및 파라미터 (실로봇 전용)

### `launch/go2_navigation_real.launch.py` (신규)
- [ ] `use_sim_time: false` 고정
- [ ] RealSense 드라이버 노드 포함 (`realsense2_camera`)
- [ ] `/clock` 퍼블리셔 제거
- [ ] `go2_nav2_params_real.yaml` 연결

### `config/go2_nav2_params_real.yaml` (신규)
- [ ] `use_sim_time: false`
- [ ] Phase B 확인 후 `odom_topic` 반영
- [ ] 초기 속도 제한 보수적 설정 (안정 확인 후 점진적으로 올림)

| 파라미터 | 시뮬 | 실로봇 초기값 |
|---------|------|------------|
| `use_sim_time` | true | **false** |
| `vx_max` | 1.0 | **0.5** |
| `vy_max` | 0.4 | **0.2** |
| `wz_max` | 1.0 | **0.5** |
| `max_accel` | [2.5, 2.5, 3.2] | **[1.5, 1.5, 2.0]** |

---

## Phase D: 통합 테스트 절차

```
1. deploy 코드 빌드 및 단독 실행 ✅ 완료 (0226)
   └─ go2_ctrl --network lo  (sim2sim, MuJoCo)
   └─ go2_ctrl --network eno1 (sim2real, 실로봇)
   └─ LT+A → FixStand, ≡ → Velocity (RL 정책)
   └─ 실로봇 보행 확인 ✅

2. cmd_vel 수동 테스트
   └─ ros2 topic pub /cmd_vel ... → 로봇 이동 확인
   └─ 0.5초 후 타임아웃 → 자동 정지 확인

3. RealSense + RTAB-Map 맵 생성
   └─ ros2 launch go2_rtabmap.launch.py use_sim_time:=false
   └─ 수동 조종하며 실내 맵 생성 → maps/rtabmap_real.db 저장

4. Nav2 자율주행 테스트
   └─ ros2 launch go2_navigation_real.launch.py
   └─ RViz에서 Goal Pose 지정 → 이동 확인

5. 속도 점진적 상향
   └─ 0.5 → 0.7 → 1.0 m/s (안정 확인 후)
```

---

## 현재 운영 기준: Go2 align depth 사용

2026-03-25 기준 현재 운영 경로는 아래와 같다.

- Go2에서 제공되는 aligned depth를 그대로 사용
- RTAB-Map 입력 토픽:
  - `/camera/color/image_raw`
  - `/camera/aligned_depth_to_color/image_raw`
  - `/camera/color/camera_info`
- `launch/go2_rtabmap_real.launch.py`와 `launch/go2_navigation_real.launch.py`의 기본값도 이 기준에 맞춰져 있음

실측 메모:

- RealSense 요청 설정은 `424x240x15`, `enable_sync:=true`, `align_depth.enable:=true`
- `align_depth.enable:=false`로 끄면 Go2 내부 `color/depth`는 각각 거의 `15Hz`로 안정적으로 나온다.
- `align_depth.enable:=true`를 켜면 Go2 내부에서는 `color`와 `aligned_depth`가 약 `10.4Hz` 수준으로 내려간다.
- 같은 aligned depth를 PC에서 수신하면 약 `8.6Hz` 수준으로 한 번 더 떨어진다.
- 즉 현재 운영에서 fps를 제한하는 핵심 원인은 원본 센서 자체가 아니라 **Go2 내부의 align 처리 + PC로의 DDS/네트워크 전달 손실**이다.
- Go2 내부 병목 후보는 크게 두 단계로 본다:
  - `librealsense` / `realsense2_camera` 내부의 **align 연산 자체**
  - align 결과를 만들기 위해 color/depth 프레임을 맞춰 기다리는 **sync 동작**
- 현재 실측만으로는 위 두 항목의 기여도를 정량 분리하지 못했고, 둘을 더 자르려면 `enable_sync:=false`, `align_depth.enable:=true` 조건의 추가 A/B 테스트가 필요하다.

운영 해석:

- "컬러 카메라 원본이 느리다"기보다 `align_depth`를 켠 파이프라인 전체의 실효 fps가 떨어지는 것으로 보는 것이 맞다.
- RTAB-Map / Nav2 관련 파라미터는 요청 15Hz가 아니라 **PC에서 실제로 받는 aligned depth 약 8Hz**를 기준으로 잡아야 한다.

아래 `0227` 내용은 당시 카메라 이슈 대응을 위해 검토했던
`align_depth:=false + raw depth/PC 정렬` 경로에 대한 **트러블슈팅 기록**이다.
현재 기본 운영 경로로 간주하지 않는다.

---

## 트러블슈팅 기록: RealSense align_depth MIPI 에러 (0227)

### 증상

`align_depth:=true` 설정 시 Color 스트림이 약 20~30초 후 반드시 끊김:

```
Right MIPI error          ← D435I 내부 MIPI 인터페이스 불안정
Asic Temperature value is not valid!  ← ASIC 상태 불안정
uvc streamer watchdog triggered on endpoint: 132  ← Color stream 끊김
```

### 시도한 해상도/fps 조합 (모두 실패)

| depth | color | align_depth | 결과 |
|-------|-------|-------------|------|
| 640×480@15fps | 640×480@15fps | true | watchdog endpoint 132 |
| 640×480@6fps | 640×480@6fps | true | watchdog endpoint 132 |
| 320×240@6fps | 320×240@6fps | true | Depth 미지원→848×480@30fps 자동 전환 후 watchdog |
| **424×240@6fps** | **424×240@6fps** | true | depth/color 모두 정상 인식, 그래도 watchdog |

### 원인 분석

**D435I 내부 데이터 흐름:**

```
┌─────────────────────────────────────────┐
│  D435I 카메라 하드웨어                    │
│                                         │
│  [IR Left]  ─┐                          │
│  [IR Right] ─┤  MIPI CSI-2  →  [ASIC]  │  ← Right MIPI error 발생 지점
│  [RGB]      ─┘                          │
│                                         │
└──────────────────────┬──────────────────┘
                       │ USB 3.2
                 [Go2 USB 포트]
                       │
              [realsense2_camera 노드]
              (align_depth 소프트웨어 처리)
                       │
                  [네트워크 ~16 MB/s, 이더넷 125 MB/s의 13%]
                       │
                      [PC]
```

- **USB / 네트워크 대역폭 문제 아님**: 424×240@6fps ≈ 3 MB/s (USB 3.2 여유), 네트워크 사용량 16 MB/s (이더넷 13%)
- **MIPI 내부 문제**: 에러 발생 지점은 카메라 센서 → ASIC 구간 (USB 이전)
- `align_depth:=true` → Sync Mode 활성화 → IR Left / IR Right / RGB 세 센서가 타이밍을 맞춰 **동시에** MIPI 버스로 프레임 전송 → MIPI 버스 부하 증가 → Right IR 신호 불안정
- `align_depth:=false` → 세 센서가 **독립적으로** 전송 → MIPI 부하 없음 → 안정적
- `align_depth` 자체는 카메라 하드웨어 처리가 아닌 **realsense2_camera 노드(Go2 CPU) 소프트웨어 처리** → CPU 부담도 존재 (Jetson 급 임베디드에서 30fps → 2~5fps로 추락하는 사례 보고됨)
- 소프트웨어(해상도/fps 조절)로는 해결 불가. Intel GitHub에 동일 에러 이슈 다수 (realsense-ros #2149, #2191, librealsense #9947)

### 당시 검토 방향: Go2에서 align_depth=false, PC에서 정렬

```
[Go2] align_depth:=false (안정적)
  → /my_go2/depth/image_rect_raw  (depth 좌표계, raw)
  → /my_go2/color/image_raw       (color 좌표계)
  → /my_go2/depth/camera_info
  → /my_go2/color/camera_info

[PC] depth_image_proc/register 노드 → aligned depth 생성

[RTAB-Map] PC에서 생성된 aligned depth 구독
```

**Go2 실행 명령어 (당시 검토안)**:
```bash
# Go2 SSH (Noetic)
roslaunch realsense2_camera rs_camera.launch camera:=my_go2 depth_width:=424 depth_height:=240 depth_fps:=6 color_width:=424 color_height:=240 color_fps:=6 align_depth:=false enable_infra1:=false enable_infra2:=false
```

> ⚠️ 당시 검토 기준에서는 `launch/go2_rtabmap_real.launch.py`의 depth 토픽 리매핑을
> `/my_go2/aligned_depth_to_color/image_raw` → `/my_go2/depth/image_rect_raw` 로 변경 필요
> (또는 PC에서 `depth_image_proc/register`로 aligned depth 생성 후 해당 토픽 사용)

---

## 트러블슈팅: RealSense Depth 4fps 한계 (Jetson + librealsense 버전 불일치) (0227)

### 원인 확인

Go2 내부는 **Jetson Orin NX (JetPack 5.1.1, L4T R35.3.1)**.
realsense2_camera 노드가 링크하는 librealsense 버전:

```bash
ldd /opt/ros/noetic/lib/librealsense2_camera.so | grep realsense
# → librealsense2.so.2.50 => /opt/ros/noetic/lib/aarch64-linux-gnu/librealsense2.so.2.50
```

| 라이브러리 | 버전 | 실제 사용 |
|-----------|------|----------|
| 시스템 librealsense2 | **2.54.2** | rs-view 등 독립 도구 |
| ros-noetic-librealsense2 | **2.50.0** | realsense2_camera 노드 ← 문제 |

librealsense **2.53.1부터** JetPack 5 (L4T R35) 패치가 포함됨.
2.50.0에서는 USB XU 커맨드(fps 설정 명령)가 실패 → `control_transfer returned error` → 기본값 4fps로 떨어짐.

---

### 당시 대안 A: realsense2_camera 소스 빌드 (librealsense 2.54.2 링크)

```bash
# Go2 Noetic 터미널 (ROS2 source 없는 상태)

# 1. 워크스페이스 생성 + 소스 클론
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros1-legacy --depth 1

# 2. 기존 ROS librealsense 제거 (빌드 시 시스템 2.54.2를 자동으로 찾게 함)
sudo apt remove ros-noetic-librealsense2 ros-noetic-realsense2-camera

# 3. 빌드
cd ~/realsense_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release

# 4. 확인 (2.54 가 나와야 성공)
ldd ~/realsense_ws/devel/lib/librealsense2_camera.so | grep realsense
```

성공 시 실행:
```bash
source ~/realsense_ws/devel/setup.bash   # 항상 이 터미널에서 실행
roslaunch realsense2_camera rs_camera.launch \
  camera:=my_go2 \
  depth_width:=424 depth_height:=240 depth_fps:=15 \
  color_width:=424 color_height:=240 color_fps:=15 \
  align_depth:=false \
  enable_infra1:=false enable_infra2:=false \
  initial_reset:=true
```

---

### 당시 대안 B: 4fps 수용 + 딜레이 최소화

RTAB-Map `DetectionRate: 1.0Hz` 설정이므로 depth 4fps로도 SLAM 동작에 지장 없음.
딜레이를 줄이려면 **설정 fps를 실제 fps에 맞추기** (버퍼 불일치 제거):

```bash
# depth_fps:=6 으로 낮추면 XU 커맨드 실패해도 버퍼 기대치와 실제 fps 간격이 줄어 딜레이 감소
roslaunch realsense2_camera rs_camera.launch \
  camera:=my_go2 \
  depth_width:=424 depth_height:=240 depth_fps:=6 \
  color_width:=424 color_height:=240 color_fps:=6 \
  align_depth:=false \
  enable_infra1:=false enable_infra2:=false \
  initial_reset:=true
```

`go2_rtabmap_real.launch.py`에서 `scan_time` 수정 필요:

| depth 실제 fps | scan_time 값 |
|---------------|-------------|
| 15fps (소스 빌드 성공 시) | `0.067` (현재값) |
| 6fps | `0.167` |
| 4fps | `0.250` |

---

## 속도 파라미터 참고

현재 `config/go2_nav2_params.yaml` 값은 **unitree_rl_lab `limit_ranges` 기준**:

```yaml
vx_max: 1.0   # limit_ranges x=±1.0
vy_max: 0.4   # limit_ranges y=±0.4
wz_max: 1.0   # limit_ranges ang_z=±1.0
```

> ⚠️ 실로봇 초기 테스트는 Phase C 표의 보수적 값으로 시작. 안정 확인 후 단계적으로 올린다.
