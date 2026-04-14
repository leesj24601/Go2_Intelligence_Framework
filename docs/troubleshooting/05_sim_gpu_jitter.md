---
tags: [project, sim, isaac-sim, slam, gpu, troubleshooting]
status: identified (sim-only, 실로봇 무관)
project: go2_intelligence_framework
type: troubleshooting
created: 2026-04-09
---

# Isaac Sim GPU 공유로 인한 관절 진동 현상

## 증상

SLAM 또는 Nav2를 시뮬 환경에서 실행하면 로봇이 정지 상태에서도 관절이 떨리고 odom_angular가 진동한다.

- Charts 탭 joint position: 정상 ±0.015 rad → SLAM 켤 때 ±0.03 rad (2배)
- odom_angular: 정상 ±0.08 rad/s @ 5~8Hz → SLAM 켤 때 ±0.15 rad/s @ 10~30Hz
- 조건: SLAM/Nav2 종료 시 즉시 정상으로 돌아옴
- cmd_vel: 아무것도 발행되지 않음 (명령 원인 아님)

## 원인

**GPU 공유 → physics step 타이밍 불균일 → RL policy 보정 오차 누적**

```
정상 (SLAM 없음):
physics substep: 32ms → 32ms → 32ms  (일정)
RL policy: 정확한 dt로 토크 계산 → 안정적

SLAM + RViz 켤 때:
RTAB-Map 카메라 처리 + RViz OpenGL 렌더링이 GPU 점유
Isaac Sim PhysX와 GPU 타임슬롯 경합
physics substep: 30ms → 34ms → 31ms  (불균일)
RL policy: dt가 일정하다고 가정 → 보정이 틀어짐 → 흔들림
```

Isaac Sim /clock 주기(31Hz)와 GPU 사용률(37%)은 안정적으로 보이지만,
**각 physics substep 내부 타이밍**이 미세하게 불규칙해지는 것이 원인이다.

## 진단 과정

```bash
# /clock 안정성 확인 → 31Hz, std dev 0.001s — 이상 없음
ros2 topic hz /clock

# GPU 사용률 확인 → RTX 4090, 37% — 이상 없음
nvidia-smi

# odom 실제 값 확인 → 진동이 실제 물리 현상임을 확인
ros2 topic echo /odom --field twist.twist.angular.z

# cmd_vel 확인 → 아무것도 없음 (명령 원인 아님)
ros2 topic echo /cmd_vel
```

odom_angular echo 비교:

| 상태 | 부호 전환 주기 | 진폭 |
|------|--------------|------|
| SLAM 없음 | 매 4~6 메시지 (5~8Hz) | ±0.05~0.09 rad/s |
| SLAM 켤 때 | 매 1~3 메시지 (10~30Hz) | ±0.05~0.17 rad/s |

## 실로봇에서 미발생 이유

실로봇 환경에서는 Isaac Sim 자체가 없다.
SLAM/Nav2는 별도 PC 또는 프로세스에서 실행되며, 로봇 RL policy의 GPU를 공유하지 않는다.
따라서 이 현상은 **시뮬레이터 고유의 한계**이며 실 배포와 무관하다.

## 완화 방법

SLAM 처리 빈도 낮추기 (`launch/go2_rtabmap.launch.py`):

```python
# localization 모드 DetectionRate 기존 2.0 → 0.5
"Rtabmap/DetectionRate": "0.5",
```

RViz에서 GPU 부하 디스플레이 비활성화:
- PointCloud2 디스플레이 끄기
- Camera Image 디스플레이 끄기

---

## 부록: robot_state_publisher orphan 누적 문제

### 현상

SLAM을 켰다 끄기를 반복하면 `robot_state_publisher`가 종료되지 않고 누적된다.

```bash
ros2 node list | grep robot_state
# /robot_state_publisher (8개 동시 실행 확인됨)
```

8개 × 12 joint × 100Hz = **9,600 TF 메시지/초** → TF 플러드 → 진동 악화

### 원인

`WebLaunchManager`가 `ros2 launch` 프로세스에 SIGKILL을 보낼 때,
`ros2 launch`만 종료되고 자식 프로세스(`robot_state_publisher`, `rtabmap` 등)는
orphan 상태로 살아남아 누적된다.

### 수정 (web_launch_manager.py)

`start_new_session=True`로 프로세스 그룹을 분리하고,
종료 시 `os.killpg()`로 그룹 전체에 시그널을 전파한다.

```python
# Popen 시 그룹 분리
process = subprocess.Popen(..., start_new_session=True)

# 종료 시 그룹 전체에 전파
os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # 정상 종료
os.killpg(os.getpgid(process.pid), signal.SIGKILL)  # 강제 종료
```

### 즉시 해결 (현재 세션)

```bash
pkill -f robot_state_publisher
```
