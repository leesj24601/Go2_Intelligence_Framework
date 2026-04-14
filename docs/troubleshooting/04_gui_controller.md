---
tags: [project, gui, controller, troubleshooting]
status: resolved
project: go2_intelligence_framework
type: troubleshooting
created: 2026-04-08
---

# GUI 컨트롤러 트러블슈팅 기록

## 실제 트러블슈팅 기록

### 1. `PySide6` 미설치

증상:

- GUI 실행 시 `PySide6` import 실패

조치:

- `python_qt_binding` 우선, `PyQt5` fallback으로 변경

결론:

- ROS2 환경에서는 이 방식이 더 현실적이었음

### 2. `run_gui_controller.sh`에서 `AMENT_TRACE_SETUP_FILES` 에러

증상:

- `set -u` 상태에서 `/opt/ros/humble/setup.bash` source 시 unbound variable 에러

조치:

- ROS setup source 구간에서만 `set +u` 처리

### 3. `use_sim_time` 파라미터 중복 선언

증상:

- `ParameterAlreadyDeclaredException`

조치:

- 이미 선언된 파라미터는 재선언하지 않도록 수정

### 4. `log_view` 생성 전 접근

증상:

- GUI 시작 시 `wrapped C/C++ object ... has been deleted` 또는 속성 접근 오류

조치:

- `feedback_label`, `log_view` 생성 순서를 waypoint list 갱신보다 앞쪽으로 조정

### 5. 수동 버튼이 너무 약하게 움직임

증상:

- 짧게 누르면 거의 안 움직여 보임

원인:

- 초기 수동 속도를 보수적으로 낮게 넣었음

조치:

- 수동 버튼 속도를 Nav2/시뮬 기준 최대값에 맞춤
- 현재 값:
  - 전후 `1.0 m/s`
  - 횡이동 `0.6 m/s`
  - 회전 `1.0 rad/s`

### 6. pose 갱신이 느리게 보임

증상:

- GUI의 `Pose:` 숫자가 실제 움직임보다 늦게 따라오는 느낌

조치:

- ROS spin 주기, 상태 갱신 주기, 수동 cmd_vel 재발행 주기를 더 촘촘하게 조정

### 7. waypoint 저장은 되는데 이동은 안 됨

초기 증상:

- GUI에서 waypoint 저장은 되지만 `Go To Selected Waypoint`가 동작하지 않음

조사 결과:

- RViz `2D Goal Pose`도 같이 안 되는 시점이 있었음
- 즉 GUI 문제가 아니라 Nav2 경로 자체가 끊겨 있었음

원인:

- `controller_server`는 `/cmd_vel_nav`를 발행
- `velocity_smoother`는 `cmd_vel_in_topic`이 지정되지 않아 입력을 받지 못함

조치:

- `config/go2_nav2_params.yaml`
- `config/go2_nav2_params_real.yaml`

두 파일의 `velocity_smoother`에 아래 설정 추가:

```yaml
cmd_vel_in_topic: "cmd_vel_nav"
```

결론:

- 이 수정은 GUI가 아니라 Nav2 설정 문제였음

### 8. direct action client / `nav2_simple_commander` 경로의 불안정성

증상:

- action server readiness 판정이 실제 동작과 어긋남
- RViz는 되는데 GUI action client는 안 되는 구간이 있었음

조치:

- GUI waypoint 입력을 RViz와 동일한 `/goal_pose` 경로로 변경

결론:

- 현재 프로젝트에서는 RViz 호환 방식이 더 안정적이었음

### 9. `Delete Waypoint` 시 Qt 객체 삭제 오류

증상:

- waypoint 삭제 후 `QListWidgetItem` 접근 시 크래시

조치:

- `item.text()`를 먼저 저장한 뒤 목록 갱신

### 10. waypoint 이동 시 회전만 하거나 방향이 어색함

증상:

- waypoint 좌표는 맞는데 회전 위주로 보이거나 저장 yaw와 다르게 끝남

경과:

- waypoint 이동을 위치 우선으로 보낼 때는 yaw가 저장값과 달라짐
- 저장 yaw까지 강하게 주면 제자리 회전 위주 증상이 생김

현재 결론:

- 지금은 RViz `2D Goal Pose`와 동일하게 저장된 `x, y, yaw` 전체를 goal로 보냄
- 최종 자세는 Nav2/MPPI/goal checker 동작의 영향을 받음
- 완전한 자세 정렬을 강제하려면 Nav2 파라미터 측 조정이 더 적합함

---
