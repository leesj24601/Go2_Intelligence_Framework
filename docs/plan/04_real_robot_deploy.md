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

