# RTAB-Map Real 구성 검토 후 수정 필요 항목

기준일: 2026-03-25

검토 대상:

- `launch/go2_rtabmap_real.launch.py`
- `src/go2_gui_controller/go2_gui_controller/odom_restamper.py`
- `config/go2_nav2_params_real.yaml`
- `docs/04_real_robot_deploy.md`
- `docs/11_rtabmap_real_topics.md`

## 1. 결론 요약

현재 `rtabmap real` 구성에서

- `static transform`
- `odom_restamper`
- `rgbd_sync`

세 요소의 **존재 이유 자체는 타당**하다.

다만 현재 설정값은 아래 세 종류가 섞여 있다.

1. 실측 기반 값
2. ROS/RealSense 표준 관례값
3. 임시로 넉넉하게 잡은 튜닝값

따라서 지금 필요한 작업은

- 구조를 갈아엎는 것보다
- **실환경 기준으로 아직 확정되지 않은 값들을 정리하고**
- **문서와 launch 간 불일치를 해소하는 것**

이다.

---

## 2. 우선 수정해야 하는 항목

### 2.1 RealSense 입력 토픽 기준 통일

상태: **정리 완료**

현재 가장 먼저 정리해야 할 부분이었고, 운영 기준 문서화는 완료했다.

운영 기준은 이제 아래처럼 보는 것이 맞다.

- **실제 운영 기준: Go2에서 align depth를 받아 사용 중**

즉 이 항목은 더 이상

- "raw depth를 쓸지 aligned depth를 쓸지 결정"

문제가 아니라,

- **실제 운영 기준에 맞지 않는 문서/주석을 정리하는 문제**

로 보는 것이 맞다.

이제 운영 기준은 아래처럼 고정한다.

#### 현재 launch 기준

- RGB: `/camera/color/image_raw`
- Depth: `/camera/aligned_depth_to_color/image_raw`
- CameraInfo: `/camera/color/camera_info`

파일:

- `launch/go2_rtabmap_real.launch.py`

#### 이전 충돌 지점

- Go2 내부 RealSense는 `align_depth:=false` 권장
- depth는 raw depth (`/my_go2/depth/image_rect_raw`) 또는
  PC에서 따로 정렬한 aligned depth를 쓰는 방향으로 적혀 있음

파일:

- `docs/04_real_robot_deploy.md`

#### 왜 수정이 필요한가

- 실제 운영에서는 이미 align depth를 사용 중인데,
  일부 문서는 여전히 raw depth / PC register 경로를 기본안처럼 설명한다.
- `rgbd_sync`는 현재 입력 토픽 전제를 그대로 사용하므로
  문서와 launch 기준이 다르면 운영/디버깅 시 혼선이 생긴다.
- 따라서 지금 필요한 것은 선택지가 아니라
  **현재 운영 기준을 저장소 전체에 명확히 반영하는 일**이다.

#### 정리 결과

운영 기준은 아래로 확정했다.

1. **Go2에서 제공되는 align depth 사용**
2. `go2_rtabmap_real.launch.py`의 현재 입력 구조를 기준 구조로 유지
3. raw depth / PC register를 기본 경로처럼 적어 둔 문서는
   "과거 검토안" 또는 "대안 경로"로 격하하거나 삭제

정리한 대상:

- `docs/04_real_robot_deploy.md`
- `docs/11_rtabmap_real_topics.md`
- `launch/go2_navigation_real.launch.py` 주석

추가로 운영 중 확인할 항목:

- 실제 운영 토픽 namespace가 `/camera/...`로 최종 고정인지
- 또는 Go2 내부 namespace만 다르고 브리지에서 `/camera/...`로 맞춰 받고 있는지

#### 우선순위

완료

---

### 2.2 `odom_restamper`의 TF 중복 발행 가능성 확인 후 구조 정리

상태: **기본값 조정 완료**

현재 `odom_restamper`는 보정된 odom뿐 아니라
`odom -> base_link` TF도 다시 발행한다.

#### 현재 동작

- 입력: `/utlidar/robot_odom`
- 출력:
  - `/utlidar/robot_odom_restamped`
  - `/tf` (`odom -> base_link`)

#### 왜 수정이 필요한가

- 원본 브리지 또는 다른 노드가 이미 `odom -> base_link`를 발행하면
  TF authority가 겹칠 수 있다.
- 배포 문서에도 원래 TF 발행 주체를 먼저 확인하라고 되어 있다.

#### 확인 후 분기

1. **원본 브리지가 TF를 발행하지 않는 경우**
   - 현재처럼 `odom_restamper`가 TF를 발행해도 됨

2. **원본 브리지가 이미 TF를 발행하는 경우**
   - 어느 쪽을 authoritative source로 쓸지 결정해야 함
   - 필요하면 원본 TF를 끄고 `odom_restamper`만 유지
   - 또는 `odom_restamper`는 topic만 내보내고 TF는 끄는 구조로 변경

#### 코드 차원에서 필요한 개선

- `publish_tf`는 이미 파라미터화되어 있으므로 유지 가능
- launch에서 이 값을 명시적으로 관리하도록 정리했고, 기본값은 `false`로 변경함
- "원본 TF 있음/없음"에 따른 운영 모드를 문서화해야 함

#### 우선순위

기본값 변경 완료, 문서화 계속 유지

---

### 2.3 `rgbd_sync` 파라미터를 실측 기준으로 재확정

상태: **2차 조정 완료**

이전 값:

- `approx_sync: true`
- `approx_sync_max_interval: 0.5`
- `queue_size: 30`

#### 왜 수정이 필요한가

- `approx_sync` 자체는 필요하다.
- 하지만 `0.5초`, `30개`는 "실환경에서 계측 후 확정한 값"이라기보다
  일단 붙게 하려고 넉넉하게 잡은 값에 가깝다.
- 동기화 허용폭이 너무 크면
  오래된 RGB/depth 조합이 묶일 위험이 있다.

#### 실측 확인 결과

2026-03-25 실측:

- `ApproximateTimeSynchronizer(slop=0.5)` 기준 매칭된 20샘플에서
  - `depth - rgb`: 평균 약 `0.0567s`
  - 최대 약 `0.0667s`
  - `camera_info - rgb`: `0.0s`

즉 현재 카메라 입력 기준에서는 `0.5s`가 필요 이상으로 넓다.

#### 수정 권장 방향

- 실제 RGB/depth/camera_info의 stamp 차이를 측정
- 그 결과를 기준으로 `approx_sync_max_interval`을 줄이는 쪽 검토
- `queue_size`도 실제 fps와 지연을 보면서 축소 검토

#### 현재 판단

- `approx_sync: true`는 유지
- `approx_sync_max_interval: 0.1`, `queue_size: 10`으로 1차 축소했을 때
  `/camera/rgbd_image`가 약 `0.4Hz` 수준으로 떨어져 실운영에서 과도한 드롭이 발생했다.
- 따라서 현재 운영값은 `approx_sync_max_interval: 0.2`, `queue_size: 20`으로 다시 완화했다.
- 이후 실제 주행 중 `/camera/rgbd_image`가 aligned depth 수신률에 가깝게 안정적으로 따라오는지 다시 확인

#### 우선순위

2차 조정 완료, 런타임 재확인 필요

---

## 3. 다음 단계로 수정해야 하는 항목

### 3.1 `base_link -> camera_link` 회전값 검증

상태: **현재 값 유지**

현재 실로봇 launch는 아래 값을 사용한다.

- translation: `(0.33, 0.0, 0.09)`
- rotation: `(0, 0, 0)`

#### 현재 판단

- 위치값 `(0.33, 0.0, 0.09)`는 주석상 수동 실측 근거가 있다.
- 하지만 회전값 `0,0,0`은 "카메라가 정확히 정면/수평 장착"이라는 가정이다.
- 현재 확인 기준으로 카메라 렌즈가 정확히 전방을 보고 있으므로, 현 시점에서는 `0,0,0` 유지 판단이 타당하다.

#### 왜 수정이 필요한가

- 카메라가 실제로 약간 하향 또는 비틀려 있으면
  RTAB-Map 정합, `/scan`, RViz 표시가 미묘하게 어긋날 수 있다.

#### 수정 권장 방향

- 실제 장착 자세를 다시 측정
- 회전이 거의 없다면 현재 값 유지
- 약간이라도 기울어져 있으면 static TF 회전값 반영

#### 우선순위

현재는 추가 수정 불필요

---

### 3.2 `odom_restamper` 발행 주기(`publish_rate_hz`) 정리

상태: **우선순위 하향**

현재 코드 기본값은 `10.0Hz`이다.

#### 왜 수정이 필요한가

- launch에서는 이 값을 명시하지 않는다.
- 실제 입력 odom 주파수와 다르면
  너무 느리게 재발행하거나
  같은 pose를 반복 송신하게 된다.
- 다만 현재 SLAM은 restamped odom topic을 직접 소비하지 않고 TF를 사용한다.
- 현재 restamped odom의 실사용처는 주로 Nav2 쪽이며, raw odom의 2024년 timestamp 문제를 피하기 위해
  `go2_navigation_real.launch.py` / `go2_nav2_params_real.yaml`는 `/utlidar/robot_odom_restamped`를 보도록 이미 조정했다.

#### 수정 권장 방향

- 실 odom 발행 주기 측정
- launch에서 `publish_rate_hz`를 명시
- 입력 주기와 같게 하거나, 최소한 의도적으로 선택한 값으로 고정

#### 우선순위

SLAM 기준 낮음, Navigation 안정화 단계에서 재검토

---

### 3.3 `depthimage_to_laserscan`의 `scan_time`를 실제 카메라 fps와 맞추기

상태: **1차 조정 완료**

이 항목은 질문 범위의 핵심 세 요소는 아니지만,
실운영에서는 같이 정리해야 한다.

#### 현재 상태

- 이전 launch 값: `0.08`
- 배포 문서에는 실제 fps에 따라 `0.067 / 0.167 / 0.250` 등 변경 필요하다고 적혀 있음

#### 왜 수정이 필요한가

- 현재 launch와 문서가 완전히 일치하지 않는다.
- Nav2 costmap이 `/scan`을 사용하므로 실사용 품질에 영향이 있다.

#### 실측 확인 결과

2026-03-25 실측:

- 요청 설정: `424x240x15`, `enable_sync:=true`, `align_depth.enable:=true`
- Go2 내부:
  - `/camera/color/image_raw`: 약 `10.4Hz`
  - `/camera/depth/image_rect_raw`: 약 `11~12Hz`
  - `/camera/aligned_depth_to_color/image_raw`: 약 `10.4Hz`
- Go2 내부에서 `align_depth.enable:=false`로 끄면:
  - `/camera/color/image_raw`: 약 `15.0Hz`
  - `/camera/depth/image_rect_raw`: 약 `15.0Hz`
- PC 수신:
  - `/camera/aligned_depth_to_color/image_raw`: 안정적일 때 약 `8.6Hz`

즉 원본 센서 자체는 15Hz 출력을 낼 수 있고,
현재 실운영 fps 저하는 **Go2 내부 align 처리**와 **PC로 전달되는 aligned depth의 추가 손실**로 해석하는 것이 맞다.

현재 Go2 내부 병목 후보는 두 단계로 나눈다.

- `librealsense` / `realsense2_camera` 내부의 **align 연산 자체**
- align 결과를 만들기 위해 color/depth 프레임을 맞춰 기다리는 **sync 동작**

현재까지의 실측은 "`align_depth.enable:=false`이면 15Hz, `align_depth.enable:=true`이면 약 10Hz"까지는 보여주지만,
위 두 후보의 기여도를 정량 분리한 상태는 아니다.
이를 더 자르려면 `enable_sync:=false`, `align_depth.enable:=true` 조합의 추가 A/B 테스트가 필요하다.

#### 현재 판단

- `scan_time`을 `0.12`로 재조정
- 기준은 요청 15Hz가 아니라 PC에서 실제 소비하는 aligned depth 약 8Hz
- 이후 실제 주행 중 `/scan` 갱신감과 Nav2 장애물 반응을 다시 확인

#### 우선순위

1차 조정 완료, 런타임 재확인 필요

---

## 4. 유지해도 되는 항목

### 4.1 `camera_link -> camera_color_optical_frame` 회전값

현재 값:

- `(-1.5708, 0, -1.5708)`

이 값은 임의값이라기보다
ROS optical frame 관례에 맞춘 표준 회전값으로 보는 것이 맞다.

따라서 특별한 프레임 정의 변경이 없다면 유지해도 된다.

---

### 4.2 `odom_restamper`의 planarization 자체

현재 처리:

- `z = 0`
- roll/pitch 제거
- yaw만 유지
- `linear.z`, `angular.x`, `angular.y` 제거

이 부분은 현재 프로젝트 목적이
"실내 평면 RTAB-Map + Nav2"라는 점을 고려하면
설계 의도는 명확하다.

다만 "Go2라서 roll/pitch를 무조건 버려도 된다"는 뜻은 아니다.

- 현재 구조에서 RTAB-Map 본체는 upstream `odom -> base_link` TF를 사용하므로
  planarized odom이 SLAM 본체의 자세 추정 경로를 직접 대체하지는 않는다.
- `/utlidar/robot_odom_restamped`는 현재 Nav2용 2D odom 성격이 강해서
  평면 실내 주행에서는 타당하다.
- 반대로 경사로, 단차, 3D 자세 제어, 자세 안정화처럼 roll/pitch가 의미 있는 경우에는
  이 planarized odom을 그대로 쓰면 위험할 수 있다.
- 다행히 raw `/utlidar/robot_odom`은 그대로 남아 있으므로
  3D 소비자는 raw odom을 보게 분리하는 것이 맞다.

즉 수정 대상은 planarization 자체보다

- TF authority 관리
- 발행 주기
- 운영 모드 문서화

쪽이다.

---

### 4.3 `rgbd_sync` 사용 자체

현재 RTAB-Map은 `subscribe_rgbd = true`로
`/camera/rgbd_image`를 직접 받는 구조다.

따라서 `rgbd_sync`를 없애는 방향보다
입력 토픽과 sync 파라미터를 정리하는 방향이 맞다.

---

## 5. 실제 수정 순서 권장안

### Step 1

실운영 카메라 토픽 기준 확정

- `/camera/...` 유지인지
- `/my_go2/...` raw depth 기반인지
- PC register 기반 aligned depth인지

먼저 하나로 결정

### Step 2

결정된 기준에 맞춰 아래 파일 동시 수정

- `launch/go2_rtabmap_real.launch.py`
- `launch/go2_navigation_real.launch.py`
- `docs/04_real_robot_deploy.md`
- `docs/11_rtabmap_real_topics.md`

### Step 3

TF 발행 주체 확인 후 `odom_restamper` 운영 방식 확정

- 원본 TF 사용 여부
- `publish_tf` 유지 여부
- 필요 시 launch 인자로 제어

### Step 4

실측 기반으로 아래 값 재설정

- `rgbd_sync.approx_sync_max_interval`
- `rgbd_sync.queue_size`
- `odom_restamper.publish_rate_hz`
- `depthimage_to_laserscan.scan_time`

### Step 5

마지막으로 static TF 재실측

- `base_link -> camera_link` translation 재확인
- 필요 시 rotation 추가

---

## 6. 바로 수정이 필요한 파일 목록

- `launch/go2_rtabmap_real.launch.py`
- `launch/go2_navigation_real.launch.py`
- `docs/04_real_robot_deploy.md`
- `docs/11_rtabmap_real_topics.md`
- `src/go2_gui_controller/go2_gui_controller/odom_restamper.py`

조건에 따라 추가:

- `config/go2_nav2_params_real.yaml`

---

## 7. 한 줄 결론

지금 당장 수정해야 하는 핵심은

1. **RealSense 입력 토픽 기준 통일**
2. **`odom_restamper`의 TF authority 정리**
3. **`rgbd_sync` 동기화 파라미터 실측 기준 재설정**

이고,

그 다음이

4. **camera static TF 회전 검증**
5. **restamper 주기 및 `/scan` 시간 파라미터 정리**

이다.
