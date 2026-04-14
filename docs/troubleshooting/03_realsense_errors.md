---
tags: [project, deploy, real-robot, realsense, troubleshooting]
status: resolved
project: go2_intelligence_framework
type: troubleshooting
created: 2026-02-27
---

# RealSense 트러블슈팅 기록


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
