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

---

## 시뮬 구조 (변경 없음)

기존 launch 파일(`go2_rtabmap.launch.py`)은 이미 외부 `/odom`을 받는 구조.
- `rtabmap_odom` 노드 없음 → 추가 불필요
- Isaac Sim OmniGraph가 `/odom` 직접 발행 → 그대로 사용
- 시뮬에서 ground truth odom으로 visual SLAM 파이프라인 검증 가능

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
