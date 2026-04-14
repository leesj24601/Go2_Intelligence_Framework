---
tags: [project, deploy, real-robot, realsense]
status: stable
project: go2_intelligence_framework
type: reference
created: 2026-03-25
---

# RealSense align_depth 운영 기준


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

