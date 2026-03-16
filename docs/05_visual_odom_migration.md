# Odometry 전략 결정: LiDAR+IMU 퓨전 (전략 변경 기록)

## 배경

### 원래 계획 (폐기)
Isaac Sim의 ground truth odom을 제거하고, `rtabmap_odom`(visual odometry)으로
대체하려 했었다.

```
RealSense RGB + Depth → rtabmap_odom (visual odometry) → /odom → rtabmap SLAM
```

### 전략 변경 이유

**왜 Visual Odometry를 쓰지 않는가?**
- 텍스처 빈약한 환경(흰 벽, 유리)에서 drift 심함
- 조명 변화에 취약
- 계산 비용 높음 (GPU 부담)

**왜 LiDAR+IMU 퓨전이 압도적으로 우월한가?**

| 항목 | Visual Odom (rtabmap_odom) | LiDAR+IMU (unitree_ros2) |
|------|---------------------------|--------------------------|
| 안정성 | 텍스처·조명 의존 | 환경 무관 |
| 정확도 | drift 수 cm~수십 cm | 오차 1cm 미만 |
| 구현 난이도 | 알고리즘 튜닝 필요 | 제조사가 이미 제공 |
| 추가 하드웨어 | 없음 | Go2 기본 탑재 4D LiDAR |

**결정적 이유**: Unitree 제조사가 `unitree_ros2` 패키지를 통해 LiDAR+IMU 퓨전으로
계산된 완성품 odom 토픽을 직접 제공한다. 별도 알고리즘 구현 불필요.

---

## 최종 아키텍처

```
[시뮬]
Isaac Sim OmniGraph (IsaacComputeOdometry)
  → /odom (ground truth)
  → rtabmap SLAM (지도: 카메라 RGB-D)

[실로봇]
Go2 4D LiDAR (Hesai / Unitree L1) + 내장 IMU
  → unitree_ros2 LIO
  → /odom (또는 /utlidar/robot_odom)
  → rtabmap SLAM (지도: RealSense RGB-D)
```

**요약**: **odom = LiDAR+IMU**, **지도 = 카메라**

### 운영 결론

- **메인 개발 경로**
  - 시뮬: ground truth `/odom`
  - 실로봇: `/utlidar/robot_odom`
- **실험 경로**
  - 시뮬 LiDAR+IMU → Point-LIO / FAST-LIO / KISS-ICP
  - 목적: 실기와 유사한 외부 odom 재현 가능성 검증

즉 현재 프로젝트의 기본 전략은
`sim = GT odom`, `real = vendor LiDAR odom`
으로 두고,
시뮬 LiDAR odom은 별도 연구 트랙으로 유지하는 것이다.

---

## 핵심 개념 구분: odom / 정합 / loop closure

세 용어는 모두 pose 추정과 관련 있지만, 역할과 시간 범위가 다르다.

### 1. odom

`odom`은 보통 `직전 시점 대비 지금 얼마나 움직였는가`를 추정한 값이다.

- 기준: 바로 이전 프레임, 이전 scan, IMU 적분
- 역할: 다음 pose를 어디쯤에서 찾아야 할지 알려주는 초기값
- 장점: 연속적이고 빠름
- 한계: 누적 drift가 생김

쉽게 말하면:

> "방금 1.0 m 앞으로 간 것 같다."

### 2. 정합 (registration / scan matching / visual matching)

정합은 현재 관측을 `이전 keyframe`, `local map`, `누적 맵`에 맞춰서
`지금 pose를 어디에 두어야 가장 일관적인가`를 찾는 과정이다.

- 기준: 현재 RGB-D / scan과 기존에 저장된 구조물
- 역할: odom이 준 초기 추정값을 실제 관측으로 수정
- 장점: odom drift를 완화 가능
- 한계: 특징점 부족, 반복 구조, motion blur, depth 노이즈에 약할 수 있음

쉽게 말하면:

> "odom은 1.0 m라고 했지만, 벽/기둥 모양을 보니 1.2 m 위치에 두는 것이 더 자연스럽다."

즉 맵을 붙일 때는 보통 `odom 값 그대로` 넣는 것이 아니라,
`odom으로 초기 추정 → 정합으로 pose 수정 → 수정된 pose에 맵 삽입` 순서로 간다.

### 3. loop closure

loop closure는 한참 전에 방문했던 장소를 다시 봤을 때,
`현재 노드`와 `과거 노드`가 사실 같은 공간이라는 제약을 추가하는 과정이다.

- 기준: 과거 keyframe / 과거 장소와 현재 관측의 유사성
- 역할: 장시간 누적된 drift를 전역적으로 다시 펴 줌
- 장점: 한 바퀴 돌고 돌아왔을 때 맵이 어긋나는 문제를 크게 줄임
- 한계: 잘못 잡히면 오히려 그래프 전체를 망칠 수 있음

쉽게 말하면:

> "지금 본 복도가 예전에 지나간 그 복도와 같은 장소이므로, 중간에 누적된 오차를 전체적으로 다시 맞춘다."

### 세 개념의 관계

실제 SLAM은 보통 아래 순서로 동작한다.

1. `odom`이 현재 위치의 초기 추정값을 준다.
2. `정합`이 현재 관측을 기존 맵/키프레임에 맞춰 pose를 수정한다.
3. 수정된 pose에 현재 관측을 맵에 누적한다.
4. 나중에 같은 장소를 다시 만나면 `loop closure`로 과거와 현재를 연결하고 그래프를 전역 최적화한다.

### 자주 헷갈리는 부분

- `odom`과 `정합`은 둘 다 pose 추정이지만 같은 것은 아니다.
- `odom`은 짧은 구간 상대이동 추정이고, `정합`은 누적된 구조와의 일관성 검사 및 보정이다.
- `loop closure`는 매 프레임 하는 보정이 아니라, 예전 장소를 다시 만났을 때 수행하는 전역 보정이다.

### 우리 프로젝트 기준 해석

- 시뮬: `odom = Isaac Sim ground truth`, `정합/loop closure = RTAB-Map RGB-D`
- 실로봇: `odom = unitree_ros2 LiDAR+IMU`, `정합/loop closure = RTAB-Map RealSense RGB-D`

즉 실로봇에서는 `LiDAR+IMU odom`이 짧은 구간 움직임을 제공하고,
RTAB-Map이 카메라 관측으로 현재 pose를 보정하며,
필요할 때 loop closure로 누적 drift를 전역적으로 줄이는 구조다.

---

## 시뮬 구조 (변경 없음)

기존 launch 파일(`go2_rtabmap.launch.py`)은 이미 외부 `/odom`을 받는 구조.
- `rtabmap_odom` 노드 없음 → 추가 불필요
- Isaac Sim OmniGraph가 `/odom` 직접 발행 → 그대로 사용
- 시뮬에서 ground truth odom으로 visual SLAM 파이프라인 검증 가능

---

## 시뮬 LiDAR odom 실험 기록

실기 Go2는 이미 `/utlidar/robot_odom`을 제공하므로,
시뮬에서도 가능한 한 비슷하게
`LiDAR + IMU -> 외부 odom -> RTAB-Map`
구조를 재현하려는 실험을 진행했다.

### 1. KISS-ICP 실험 결과

시도한 구조:

```text
/utlidar/cloud (Isaac Sim raw PointCloud2)
  -> KISS-ICP
  -> /utlidar/robot_odom
```

결과:
- 입력 토픽 구독은 정상
- 그러나 odom이 수십~수백 m 단위로 급격히 튀는 발산 발생
- `odom -> base_link` TF도 순간적으로 큰 오차를 보이며 RTAB-Map 맵이 멀리 떨어진 위치에 새로 생김

해석:
- Isaac Sim raw cloud는 KISS-ICP가 기대하는 일반적 회전형 LiDAR 입력과 완전히 같지 않았다.
- self-hit, quadruped의 roll/pitch/z motion, 초기 registration 불안정이 겹치면 KISS가 pose를 잘못 누적했다.
- 즉 **KISS-ICP는 현재 우리 Isaac Sim 입력에서 안정적인 기본 경로로 채택하지 않음**.

### 2. Point-LIO / FAST-LIO 계열 실험 결과

시도한 구조:

```text
/utlidar/cloud_timed + /imu/data
  -> Point-LIO 계열
  -> /utlidar/robot_odom
```

여기서 `/utlidar/cloud_timed`는 기본 Isaac `/utlidar/cloud`가 아니라,
`x, y, z, intensity, ring, time`을 갖도록 별도 가공한 PointCloud2이다.

결과:
- `/utlidar/robot_odom` 생성 자체는 성공
- `odom -> base_link` TF도 생성 가능
- 그러나 초기에는
  - 입력 point cloud 가공 비용이 너무 무거움
  - sim FPS / IMU / LiDAR rate 저하
  - 정지 상태에서도 odom drift / yaw jitter
  - RTAB-Map 맵 중첩 및 ghost layer
  가 발생했다.

후속 조치:
- `cloud_timed` downsample
- per-frame output 사용
- Point-LIO 파라미터 조정
- raw odom을 2D filtered odom으로 재발행하여 RTAB-Map에는 planar pose만 제공

개선 효과:
- `/utlidar/robot_odom_raw` 대비 `/utlidar/robot_odom`은
  - z 제거
  - roll/pitch 제거
  - yaw/xy jitter 감소
- 정지 상태 odom 안정성은 유의미하게 개선

남은 한계:
- Isaac Sim에서 `time/ring/intensity` 있는 cloud를 만들기 위해
  Python 경유 custom publisher가 필요했고, 이것이 성능 병목이 되었다.
- 실기처럼 드라이버 단계에서 바로 LIO-friendly 입력을 받는 구조가 아니므로,
  **시뮬에서 LIO를 메인 개발 경로로 유지하기엔 비용이 큼**.

### 실패 원인 정리

시뮬 LiDAR odom 실험이 메인 경로로 채택되지 않은 이유는 아래와 같다.

1. **입력 형식 차이**
   - 실기 Go2는 드라이버가 LIO 친화적인 데이터(`time/ring/intensity + IMU`)를 제공
   - Isaac Sim 기본 cloud는 그렇지 않아 별도 변환 필요

2. **브리지 성능 병목**
   - `cloud_timed`를 Python에서 재패킹하면서 sim tick이 느려짐
   - 결과적으로 LiDAR/IMU/odom 전체 주기가 흔들림

3. **정지 상태 pose jitter**
   - odom이 정지 중에도 수 cm, 수 deg 흔들리면 RTAB-Map이 같은 구조를 여러 위치에 삽입
   - 이 현상은 단순 화질 저하가 아니라 **ghost layer / map island**의 직접 원인

4. **시뮬-실기 역할 차이**
   - 실기는 이미 `/utlidar/robot_odom`이 존재
   - 시뮬은 odom 생성기까지 직접 구현해야 하므로 복잡도와 비용이 더 큼

### 최종 판단

- **시뮬 메인 경로**는 GT odom으로 유지
- **실로봇 메인 경로**는 `/utlidar/robot_odom`
- Point-LIO / FAST-LIO / KISS-ICP는
  **시뮬 LiDAR odom 연구용 실험 경로**로만 유지

즉 시뮬에서는
`RTAB-Map / Nav2 / TF / GUI / launch 구조 검증`
에 집중하고,
실제 odom 품질과 ghost 여부는
실로봇 `/utlidar/robot_odom`으로 최종 검증하는 전략을 채택한다.

---

## 실로봇 적용 시 변경 사항

### odom 토픽

| 항목 | 시뮬 | 실로봇 |
|------|------|--------|
| odom 소스 | Isaac Sim OmniGraph (ground truth) | unitree_ros2 LIO (LiDAR+IMU) |
| odom 토픽 | `/odom` | `/odom` 또는 `/utlidar/robot_odom` (연결 후 확인) |
| use_sim_time | `true` | `false` |
| depth 토픽 | `/camera/depth/image_rect_raw` | `/camera/aligned_depth_to_color/image_raw` |
| camera_info | `/camera/camera_info` | `/camera/color/camera_info` |

### 실로봇 연결 후 확인 절차

```bash
# 1. unitree_ros2 브릿지 실행 후 토픽 확인
ros2 topic list | grep -E "odom|utlidar"

# 2. 실제 토픽명 확인
ros2 topic echo /odom --once
ros2 topic echo /utlidar/robot_odom --once   # 없으면 위 토픽 사용

# 3. TF 트리 확인 (odom → base_link 프레임 발행 여부)
ros2 run tf2_ros tf2_monitor
```

토픽명이 `/utlidar/robot_odom`이면 launch 파일 remapping에 추가:
```python
("odom", "/utlidar/robot_odom"),
```

---

## 파일 구성

```
launch/
├── go2_rtabmap.launch.py          # 시뮬용 (use_sim_time=true 기본값, 변경 없음)
└── go2_rtabmap_real.launch.py     # 실로봇용 (작성 예정, 04_real_robot_deploy.md 참고)
```

---

## 실행 명령어

```bash
# 시뮬 (변경 없음)
ros2 launch launch/go2_rtabmap.launch.py

# 실로봇 (작성 후)
ros2 launch launch/go2_rtabmap_real.launch.py
```
