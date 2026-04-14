# 프로젝트 현황 요약

> 최종 업데이트: 2026-04-09

---

## 1. 프로젝트 개요

**목표**: Unitree Go2 사족보행 로봇의 실내 자율 내비게이션 구현 (Sim → Real 배포)

| 항목 | 내용 |
|------|------|
| 시뮬레이션 | NVIDIA Isaac Sim 5.1.0 + Isaac Lab v2.3.0 |
| 미들웨어 | ROS2 Humble |
| SLAM | RTAB-Map (RGB-D 기반 3D/2D 맵 생성) |
| 내비게이션 | Nav2 (MPPI controller + BT navigator) |
| 보행 제어 | unitree_rl_lab (RL 정책 기반, obs ~45dim) |
| 센서 (실로봇) | RealSense D435i + Go2 내장 4D LiDAR + IMU |

---

## 2. 개발 현황

### 완료된 작업

| Phase | 내용 | 참조 문서 |
|-------|------|---------|
| Phase 0-2 | Isaac Sim 환경 구축, RL 정책 로딩, RTAB-Map SLAM 구현 | [01_rtabmap_slam_plan](01_rtabmap_slam_plan.md) |
| Phase 3-5 | Nav2 자율주행 통합 (시뮬), MPPI 설정, BT navigator | [03_nav2_plan](03_nav2_plan.md) |
| 보조 | RViz2 Go2 로봇 모델 시각화 (joint_states 연동) | [06_rviz_robot_model_plan](06_rviz_robot_model_plan.md) |
| 보조 | Odometry 전략 확정 (LiDAR+IMU, Visual Odom 폐기) | [05_visual_odom_migration](05_visual_odom_migration.md) |
| 보조 | GUI 컨트롤러 기본 기능 (버튼, waypoint, 자연어) | [plan/07_gui_controller_plan](plan/07_gui_controller_plan.md) |
| 보조 | RL 정책 전환 (IsaacLab 공식 → unitree_rl_lab) | [plan/02_policy_decision](plan/02_policy_decision.md) |
| Phase 7 | HTML 웹 GUI 컨트롤러 구현 완료 (FastAPI + WebSocket + index.html, STT 포함) | [plan/10_html_gui_controller_plan](plan/10_html_gui_controller_plan.md) |

### 진행 중인 작업

#### Phase 6: 실로봇 배포 (`04_real_robot_deploy.md`)

| 항목 | 상태 | 내용 |
|------|------|------|
| cmd_vel 수신 구현 | 미완료 | unitree_rl_lab deploy `observations.h` 수정 필요. CMakeLists에 rclcpp 추가, cmd_vel 수신 스레드 구현 |
| RealSense 설정 | 분석 중 | `align_depth:=true` 고정 운영. Go2 내부 ~10.4Hz → PC 수신 ~8.6Hz 병목 원인 조사 중 |
| Nav2 실로봇 파라미터 | 작업 중 | `go2_nav2_params_real.yaml` 최종화 필요 |

#### RTAB-Map 실로봇 튜닝 (`12_rtabmap_real_review_actions.md`)

| 항목 | 상태 | 내용 |
|------|------|------|
| RealSense 토픽 기준 통일 | 완료 | aligned depth 기반 통일 |
| odom_restamper TF 중복 | 완료 | `publish_tf:=false` 기본값 적용 |
| rgbd_sync 파라미터 조정 | 완료, 재확인 필요 | `approx_sync_max_interval` 0.5s → 0.2s, `queue_size` 30 → 20 |
| depthimage_to_laserscan | 재확인 필요 | `scan_time` 실로봇 fps 기준 재설정 |

#### GUI 컨트롤러 (`plan/07_gui_controller_plan.md`, `plan/10_html_gui_controller_plan.md`)

| 항목 | 상태 |
|------|------|
| 수동 버튼 제어 (WASD) | 완료 |
| Waypoint 저장 및 이동 | 완료 |
| 자연어 명령 파싱 (거리/방향/각도) | 완료 |
| 음성 STT (faster-whisper) 통합 | 완료 |
| HTML 웹 GUI (`web_app.py` + `index.html`, FastAPI/WebSocket) | 완료 |
| `web_launch_manager.py` (subprocess 기반, Qt 제거) | 완료 |
| `launch/go2_web_controller.launch.py` | 완료 |

---

## 3. 주요 기술 결정사항

| 항목 | 선택 | 대안 | 결정 이유 |
|------|------|------|---------|
| SLAM | **RTAB-Map** | ORB-SLAM3 | RGB-D Loop Closure 강력, 실내 환경 강인, 커뮤니티 지원 |
| RL 정책 | **unitree_rl_lab** | IsaacLab 공식 정책 | 실로봇 배포 파이프라인 내장, obs space 최소화 (~45dim) |
| Odometry | **LiDAR+IMU (unitree_ros2)** | Visual Odometry | Go2 내장 4D LiDAR 활용, 텍스처 무관, 정확도 우수 |
| Local Planner | **MPPI** | DWB, RPP | Omnidirectional 지원, 복잡 환경 강인, 사족보행 로봇 적합 |
| Goal 입력 방식 | **/goal_pose Topic** | NavigateToPose Action | RViz 호환, 간단한 구현 |
| 시뮬 센서 인터페이스 | **OmniGraph** | rclpy 직접, TCP 소켓 | 시뮬 tick 동기화, 별도 노드 불필요 |

---

## 4. 알려진 이슈

| 이슈 | 심각도 | 내용 |
|------|--------|------|
| **cmd_vel 수신 미구현** | 높음 | Nav2 → 실로봇 이동 불가. Phase 6의 핵심 블로커 |
| **RealSense fps 병목** | 중간 | align_depth 활성화 시 ~8Hz로 저하. MIPI 불안정 + sync 동작 + 네트워크 손실 복합 원인 |
| **rgbd_sync 안정성** | 중간 | `approx_sync_max_interval` 조정 후 실측 재확인 필요 |
| **static TF 회전값** | 낮음 | 실로봇 환경에서 재실측 필요 |

---

## 5. 시스템 토폴로지

```
[Isaac Sim / 실로봇]
    │
    ├─ RGB-D 이미지 (RealSense D435i)
    │     └─ /camera/color/image_raw
    │     └─ /camera/aligned_depth_to_color/image_raw
    │
    ├─ Odometry
    │     └─ /odom (unitree_ros2: LiDAR+IMU 퓨전)
    │
    └─ Joint States
          └─ /joint_states

[ROS2 파이프라인]
    │
    ├─ odom_restamper → 2D odom (planarization)
    ├─ rgbd_sync → /camera/rgbd_image (동기화)
    ├─ depthimage_to_laserscan → /scan (2D 라이다)
    │
    ├─ RTAB-Map → /map (2D 점유 격자), /rtabmap/mapData (3D)
    ├─ Nav2 → /cmd_vel (경로 계획)
    └─ GUI Controller → /goal_pose, /cmd_vel (수동/자동 목표)
```

---

## 6. 다음 우선순위

1. **cmd_vel 수신 구현** — unitree_rl_lab deploy C++ 수정 (`plan/04_real_robot_deploy.md`)
2. **RealSense fps 원인 분리** — align_depth A/B 테스트
3. **실로봇 주행 테스트** — 저속(0.2m/s)부터 점진적 검증
4. ~~HTML GUI 컨트롤러 전환~~ — **완료** (Phase 7, `web_controller` entry point 배포 가능)

---

## 7. 장기 방향

### Phase 7: HTML 기반 GUI 컨트롤러 대체 — **완료**

PyQt5 기반 GUI를 웹 브라우저에서 동작하는 HTML/JS UI로 대체 완료.

| 항목 | 내용 |
|------|------|
| 목표 | Qt 의존성 제거, 모바일/원격 접근 지원, 개발 생산성 향상 |
| 아키텍처 | FastAPI(WebSocket) 백엔드 + 단일 HTML 프론트엔드 |
| 구현 파일 | `web_app.py`, `web_launch_manager.py`, `web/index.html`, `launch/go2_web_controller.launch.py` |
| 상태 | **완료** |
| 실행 | `ros2 run go2_gui_controller web_controller --ros-args -p mode:=sim\|real` |
| 참조 | [plan/10_html_gui_controller_plan](plan/10_html_gui_controller_plan.md) |

### 미래 연구 방향

| 방향 | 상태 | 참조 |
|------|------|------|
| Semantic Navigation ("소파로 가" 등 의미 기반 명령) | 연구용 | [08_semantic_navigation_direction](08_semantic_navigation_direction.md) |
| 복층/계단 내비게이션 | 연구용 | [09_multifloor_stair_navigation_direction](09_multifloor_stair_navigation_direction.md) |

---

## 8. 관련 문서 인덱스

### archive/

| 문서 | 종류 | 상태 |
|------|------|------|
| [00_archive_orb_slam3_task](archive/00_archive_orb_slam3_task.md) | 보관 | 폐기 |

### plan/

| 문서 | 종류 | 상태 |
|------|------|------|
| [01_rtabmap_slam_plan](plan/01_rtabmap_slam_plan.md) | 구현 계획 | 완료 |
| [02_policy_decision](plan/02_policy_decision.md) | 결정 기록 | 완료 |
| [03_nav2_plan](plan/03_nav2_plan.md) | 구현 계획 | 완료 |
| [04_real_robot_deploy](plan/04_real_robot_deploy.md) | 배포 가이드 | 진행 중 |
| [05_visual_odom_migration](plan/05_visual_odom_migration.md) | 결정 기록 | 완료 |
| [06_rviz_robot_model_plan](plan/06_rviz_robot_model_plan.md) | 구현 계획 | 완료 |
| [07_gui_controller_plan](plan/07_gui_controller_plan.md) | 구현 계획 | 완료 |
| [08_semantic_navigation_direction](plan/08_semantic_navigation_direction.md) | 방향 연구 | 연구용 |
| [09_multifloor_stair_navigation_direction](plan/09_multifloor_stair_navigation_direction.md) | 방향 연구 | 연구용 |
| [10_html_gui_controller_plan](plan/10_html_gui_controller_plan.md) | 구현 계획 | **완료** |

### reference/

| 문서 | 종류 | 상태 |
|------|------|------|
| [01_rtabmap_slam_presentation](reference/01_rtabmap_slam_presentation.md) | 발표 자료 | 완료 |
| [02_project_dependencies](reference/02_project_dependencies.md) | 참조 | 활성 |
| [03_rtabmap_real_topics](reference/03_rtabmap_real_topics.md) | 참조 | 완료 |
| [04_realsense_operating_basis](reference/04_realsense_operating_basis.md) | 참조 | 활성 |

### troubleshooting/

| 문서 | 종류 | 상태 |
|------|------|------|
| [01_rtabmap_real_review_actions](troubleshooting/01_rtabmap_real_review_actions.md) | 액션 플랜 | 진행 중 |
| [02_rviz_robot_model](troubleshooting/02_rviz_robot_model.md) | 트러블슈팅 | 완료 |
| [03_realsense_errors](troubleshooting/03_realsense_errors.md) | 트러블슈팅 | 활성 |
| [04_gui_controller](troubleshooting/04_gui_controller.md) | 트러블슈팅 | 활성 |
