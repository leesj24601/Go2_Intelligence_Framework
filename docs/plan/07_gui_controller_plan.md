---
tags: [project, plan, gui, navigation]
status: in-progress
project: go2_intelligence_framework
type: implementation-plan
created: 2026-02-24
---

# Implementation Plan: Go2 GUI Controller for Button and Natural-Language Navigation

## Goal
현재 Nav2는 RViz의 `2D Goal Pose` 입력에 의존한다.
07단계의 목표는 이를 대체하거나 보완하는 GUI Controller를 만들어,

- 버튼 클릭으로 자주 쓰는 동작을 즉시 실행하고
- 자연어 입력으로 목표를 해석해 Nav2 명령으로 변환하며
- 최종적으로 시뮬레이션과 실로봇에서 동일한 제어 UX를 제공하는 것

이다.

핵심은 "GUI가 직접 로봇을 저수준 제어"하는 것이 아니라,
**Nav2의 Action/Topic 인터페이스를 감싸는 상위 제어 계층**으로 동작하게 만드는 것이다.

> 이 문서는 초기 계획서로 시작했지만, 현재는 **실제 구현 결과 + 트러블슈팅 기록**까지 반영한 작업 문서다.

---

## 현재 구현 결과

현재 저장소에는 `src/go2_gui_controller/` 패키지가 추가되어 있고,
최소 동작 가능한 GUI controller가 구현되어 있다.

현재 동작하는 기능:

- 수동 버튼 제어
  - `Forward`, `Back`, `Left`, `Right`, `Turn Left`, `Turn Right`, `Stop`
- 현재 위치 waypoint 저장
- waypoint 선택 이동
- waypoint 이름 변경 / 삭제
- 텍스트 명령 실행
- 음성 전사 입력 테스트용 `Run Voice Route`
- 현재 pose / nav 상태 / 마지막 결과 표시
- 내부 명령 로그 표시

현재 실제 동작 방식:

```text
GUI
  |- 수동 버튼 -> /cmd_vel 직접 발행
  |- waypoint 이동 -> /goal_pose 발행 (RViz 2D Goal Pose와 동일한 경로)
  |- 텍스트 명령 -> parser -> waypoint goal 또는 상대 이동 goal
  |- 음성 전사 테스트 -> parser -> 짧은 명령은 cmd_vel, waypoint/명시 거리각은 nav goal
  |- 현재 pose 표시 -> tf(map->base_link 우선) / /odom fallback
```

즉 waypoint 이동은 현재 **`/goal_pose`를 발행하는 RViz 호환 방식**으로 구현되어 있다.

---

## 빌드 / 실행

### 권장 방식

가장 덜 헷갈리고 현재 구조에서 가장 안정적인 방식은
**외부 GUI workspace에서 한 번 빌드하고, repo에서는 실행만 하는 것**이다.

`[권장]`

```bash
cd ~/Desktop/sj/go2_gui_controller_ws
colcon build --packages-select go2_gui_controller

cd ~/Desktop/sj/go2_intelligence_framework
bash ./scripts/run_gui_controller.sh
```

의미:

- 첫 두 줄: 외부 workspace에서 GUI 패키지 빌드
- 마지막 줄: repo 쪽 실행 스크립트로 GUI 실행
- `run_gui_controller.sh`는 외부 workspace의 `install/go2_gui_controller`를 우선 사용한다.

### 빌드는 언제 필요한가

빌드는 **최초 1회만** 하면 된다.

다만 아래 경우에는 다시 빌드해야 한다.

- `src/go2_gui_controller/` 내부 Python 코드가 바뀐 경우
- `package.xml`, `setup.py`, launch/config가 바뀐 경우

즉 **코드 변경이 없으면 다시 빌드할 필요 없다.**

즉 평소 사용 흐름은 아래에 가깝다.

1. 처음 한 번 빌드
2. 그 뒤에는 `bash ./scripts/run_gui_controller.sh`만 반복 실행

### 외부 workspace 구조

```text
~/Desktop/sj/go2_gui_controller_ws/
  src/
    go2_gui_controller -> ~/Desktop/sj/go2_intelligence_framework/src/go2_gui_controller
  build/
  install/
  log/
```

즉 GUI 패키지 소스는 이 repo에 두고,
`build / install / log` 산출물만 외부 workspace에 만든다.

현재 외부 workspace는 `colcon build --packages-select go2_gui_controller`의
기본 `isolated install` 구조를 사용한다.

### 직접 실행 방식

`[대안: 디버깅용 직접 실행]`

아래 방식은 디버깅용이다.

```bash
cd ~/Desktop/sj/go2_gui_controller_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run go2_gui_controller gui_controller
```

정리:

- 평소에는 `bash ./scripts/run_gui_controller.sh` 실행만 하면 된다.
- 빌드는 `~/Desktop/sj/go2_gui_controller_ws`에서 처음 한 번, 그리고 코드가 바뀌었을 때만 다시 한다.
- 즉 repo 루트에는 더 이상 GUI용 `build / install / log` 산출물을 만들지 않는다.

일상 사용 기준으로는 위보다 `bash ./scripts/run_gui_controller.sh`를 권장한다.

### 음성(STT) 준비 메모

현재 GUI에는 실제 마이크 STT를 붙이기 전 단계로,
**전사된 문자열을 직접 넣어 음성 명령 경로를 테스트하는 `Run Voice Route` UI**가 들어가 있다.

현재 권장 방향:

- ROS2 GUI는 conda가 아니라 **시스템 Python (`/usr/bin/python3`)** 기준으로 유지
- STT 엔진은 `faster-whisper`
- 실제 마이크 입력에는 `sounddevice`
- 기본 모델은 `base`

설치 예시는 아래와 같다.

```bash
sudo apt update
sudo apt install libportaudio2 portaudio19-dev

/usr/bin/python3 -m pip install --user faster-whisper sounddevice
```

설명:

- `faster-whisper`: Whisper 기반 STT 엔진
- `sounddevice`: 마이크 입력용 Python 패키지
- `libportaudio2`, `portaudio19-dev`: `sounddevice`가 필요로 하는 시스템 라이브러리

검증 명령:

```bash
/usr/bin/python3 -c "import faster_whisper, sounddevice; print('ok')"
```

주의:

- 현재 GUI 실행은 `scripts/run_gui_controller.sh` 기준으로 `/opt/ros/humble` + `/usr/bin/python3` 경로를 사용한다.
- 따라서 STT 관련 Python 패키지도 **같은 Python 환경**에 설치해야 한다.
- 개인 절대경로를 코드에 하드코딩하지 말고, 모델은 `WhisperModel(\"base\")` 같은 **모델 이름 기반 자동 다운로드** 방식으로 두는 편이 GitHub 공유에 유리하다.

선택적 환경변수:

```bash
export GO2_STT_MODEL=base
export GO2_STT_DEVICE=auto
export GO2_STT_COMPUTE_TYPE=default
export GO2_STT_DOWNLOAD_ROOT=/some/cache/path
```

설명:

- `GO2_STT_MODEL`: 예: `base`, `small`
- `GO2_STT_DEVICE`: 예: `auto`, `cpu`, `cuda`
- `GO2_STT_COMPUTE_TYPE`: 예: `default`, `int8`, `float16`
- `GO2_STT_DOWNLOAD_ROOT`: 모델 자동 다운로드 위치를 직접 지정할 때 사용

### 음성 경로 테스트 방법

실제 STT 연결 전에는 GUI 하단에서 아래처럼 테스트한다.

1. 왼쪽 입력칸에 전사 문자열을 직접 입력
2. `Run Voice Route` 클릭
3. `Feedback` / 로그창으로 결과 확인

예시:

- `앞으로` -> 짧은 `cmd_vel`
- `우회전` -> 짧은 회전 `cmd_vel`
- `앞으로 1미터 가` -> 상대 이동 nav goal
- `오른쪽으로 90도 돌아` -> 상대 회전 nav goal
- `home으로 가` -> 저장된 waypoint가 있으면 waypoint nav goal

현재 구현에서는 `Listen Voice Command`가 실제 `faster-whisper` + `sounddevice` 경로를 사용한다.
필수 라이브러리가 없으면 버튼이 비활성화되고, 모델 다운로드/마이크 오류는 GUI `Feedback` 및 로그창에 표시된다.

---

## 구현 상태 체크리스트

### 완료

- [x] `go2_gui_controller` ROS2 Python 패키지 생성
- [x] Qt 기반 데스크톱 GUI 구성
- [x] 수동 버튼 제어 (`Forward`, `Back`, `Left`, `Right`, `Turn Left`, `Turn Right`, `Stop`)
- [x] `/cmd_vel` 직접 발행
- [x] press-and-hold 방식 수동조작
- [x] 정지 버튼 최우선 처리
- [x] 수동조작 시작 시 Nav2 goal 취소
- [x] waypoint YAML 로드
- [x] 현재 위치 waypoint 저장
- [x] waypoint 선택 이동
- [x] waypoint 이름 변경 / 삭제
- [x] 텍스트 입력창 / 실행 버튼
- [x] 숫자/단위 파싱
- [x] 방향어 파싱
- [x] 회전어 파싱
- [x] waypoint 이름 기반 자연어 이동
- [x] 파싱 실패 메시지 표시
- [x] 현재 pose 조회 (`tf2` 우선, `/odom` fallback)
- [x] `MOVE_RELATIVE`, `ROTATE_RELATIVE`를 목표 pose로 변환
- [x] Isaac Sim + Nav2에서 GUI 기본 동작 테스트
- [x] Nav2 설정 문제(`cmd_vel_nav -> velocity_smoother`) 트러블슈팅 및 수정

### 대체 구현

- [x] waypoint goal 입력
  - 초기 계획: `NavigateToPose` action client
  - 최종 구현: RViz와 동일한 `/goal_pose` 발행 방식

### 부분 완료

- [~] Nav 상태 / 현재 pose / 로그 표시
  - 현재 pose, nav 상태, 내부 로그는 표시됨
  - 최종 goal result 표시 품질은 추가 보강 여지 있음
- [~] goal 전송 / 취소 처리
  - 전송과 취소는 구현됨
  - action-level feedback/result 추적은 현재 구조에서 제한적
- [~] 실로봇 대응 파라미터 정리
  - `go2_nav2_params_real.yaml`에는 반영
  - 실제 실로봇 검증은 아직 안 함

### 미구현

- [ ] 자유형 자연어 이해
  - 예: `문 앞까지 가`, `책상 오른쪽으로 가`
- [ ] 속도 수식어 자연어
  - 예: `천천히 가`, `빠르게 가`
- [ ] 자연어 명령 성공/실패 체계적 로그 저장
- [ ] waypoint 이동 성공률 측정
- [ ] 비상정지 절차 문서화
- [ ] 실로봇 GUI 운용 검증

---

## 범위

이번 단계에서 포함할 기능:

1. 버튼 기반 제어
   - 전진 / 후진 / 좌이동 / 우이동 / 좌회전 / 우회전
   - 정지
   - 저장된 waypoint 이동

2. 자연어 기반 제어
   - "앞으로 1미터 가"
   - "왼쪽으로 90도 돌아"
   - "저장한 지점으로 가"
   - "입구 지나서 오른쪽 지점으로 가"

3. 상태 표시
   - Nav2 연결 상태
   - 현재 pose
   - goal 진행 상태
   - 성공 / 실패 / 취소 결과

이번 단계에서 제외할 것:

- 완전한 음성 인터페이스
- 고급 멀티턴 대화 에이전트
- 지도 편집기 자체 구현
- Isaac Sim UI에 직접 내장하는 커스텀 패널

---

## 현재 구조와 연결 지점

현재 프로젝트의 제어 체인은 아래와 같다.

```text
RViz 2D Goal Pose
   |
   v
Nav2 (planner / controller / bt_navigator)
   |
   +--> /goal_pose 또는 NavigateToPose Action
   +--> /cmd_vel
   |
   v
Isaac Sim or Real Robot
```

GUI Controller가 추가되면 구조는 아래처럼 바뀐다.

```text
GUI App
  |- Button Panel
  |- Text Command Input
  |- Status View
  |
  +--> GUI Controller Node
         |- waypoint registry
         |- natural language parser
         |- nav2 goal bridge
         |- manual command publisher
         |
         +--> /goal_pose
         +--> /cmd_vel
```

즉 GUI는 다음 2가지 경로를 가진다.

1. 위치 기반 명령
   - Nav2 goal 발행
   - waypoint 선택
   - 자연어를 pose 또는 waypoint로 변환

2. 직접 조작 명령
   - 짧은 구간 수동 이동
   - 회전 / 정지
   - 필요 시 `/cmd_vel` 직접 발행

---

## 설계 원칙

### 1. RViz 의존 제거, Nav2 의존 유지

GUI는 RViz를 대체하지만 Nav2를 우회하지 않는다.
경로 계획과 장애물 회피는 계속 Nav2가 맡고,
GUI는 목표를 더 편하게 입력하는 계층으로 둔다.

### 2. 자연어는 2단계 파이프라인으로 구현

자연어 입력은 처음부터 LLM 하나에 전부 맡기지 않는다.

1. 규칙 기반 parser
   - 거리, 방향, 회전, 저장된 장소 이름
   - 예: "앞으로 1미터", "왼쪽 90도", "L로 가"

2. 확장형 semantic parser
   - 모호한 표현
   - 문맥 기반 위치 표현
   - 필요 시 외부 LLM 또는 별도 추론 모듈 연결

이렇게 해야 오프라인에서도 기본 명령이 동작하고, 디버깅도 쉽다.

### 3. 안전한 명령 우선순위 유지

직접 조작(`/cmd_vel`)과 Nav2 goal은 동시에 활성화되면 안 된다.

- 수동 조작 시작 시 현재 Nav2 goal 취소
- Nav2 goal 실행 중에는 수동 버튼 비활성화 또는 confirm 처리
- 정지 버튼은 언제나 최우선

### 4. 시뮬과 실로봇 공용 아키텍처

GUI 자체는 Isaac Sim 전용으로 만들지 않는다.
토픽명과 액션 인터페이스를 공통으로 맞춰,

- 시뮬: `launch/go2_navigation.launch.py`
- 실로봇: `launch/go2_navigation_real.launch.py`

둘 다 같은 GUI를 붙일 수 있게 한다.

---

## 제안 구현 구조

07단계 GUI controller는 전용 ROS2 Python 패키지로 분리했고,
현재는 워크스페이스의 `src/` 아래에 두는 형태로 정리했다.

### 실제 구현된 패키지 구조

```text
src/
  go2_gui_controller/
    package.xml
    setup.py
    resource/
    go2_gui_controller/
      __init__.py
      commands.py
      gui_app.py
      navigator_bridge.py
      manual_control.py
      waypoint_registry.py
      text_command_parser.py
      state_bridge.py
    launch/
      go2_gui_controller.launch.py
    config/
      waypoints.yaml
```

### 실제 파일 역할

- `gui_app.py`
  - GUI 렌더링
  - 버튼 이벤트
  - 텍스트 입력창
  - 상태 패널
  - 로그창
  - waypoint 저장 / 삭제 / 이름 변경

- `navigator_bridge.py`
  - waypoint / 상대 이동 goal을 Nav2로 전달
  - 현재 구현은 `/goal_pose` 발행 기반
  - goal 취소 요청 처리

- `manual_control.py`
  - `/cmd_vel` 발행
  - press-and-hold 제어

- `waypoint_registry.py`
  - 이름과 pose 매핑
  - YAML 저장 / 삭제 / 이름 변경

- `text_command_parser.py`
  - 자연어를 구조화 명령으로 변환
  - 예: `MOVE_RELATIVE`, `ROTATE_RELATIVE`, `NAVIGATE_TO_WAYPOINT`

- `commands.py`
  - 내부 명령 enum / dataclass 정의

- `state_bridge.py`
  - TF 또는 `/odom` 기반 pose 표시
  - 현재 상태 모델 보관

---

## 실제 구현 선택

초기 설계에서는 `PySide6 + nav2_simple_commander`를 유력 후보로 봤다.
하지만 실제 구현 과정에서 아래 이유로 구조를 조정했다.

### GUI 프레임워크

최종 구현:

- `python_qt_binding` 우선
- `PyQt5` fallback

이유:

- 현재 환경에 `PySide6`가 기본 설치되어 있지 않았음
- ROS2 환경에서는 `python_qt_binding` / `PyQt5`가 즉시 사용 가능했음
- 실제 실행 가능성이 중요했기 때문에 환경 적합성을 우선함

### Nav2 연동 방식

최종 구현:

- waypoint 이동은 `/goal_pose` 발행 방식
- 즉 RViz `2D Goal Pose`와 동일한 입력 경로 재사용

이유:

- `nav2_simple_commander` / action client 직접 호출 방식은 디버깅 과정에서
  현재 세션의 Nav2 상태와 미묘하게 어긋나는 경우가 있었음
- 반면 RViz의 `2D Goal Pose`는 이미 동작 검증이 가능한 경로였음
- 따라서 GUI도 **RViz와 같은 방식으로 goal을 발행**하는 쪽이 실용적이었음

---

## GUI 기술 선택

### 계획 단계의 권장안

계획 단계에서는 `PySide6 + rclpy`를 우선안으로 두었다.

하지만 실제 구현은 아래처럼 바뀌었다.

### 최종 구현안

- Qt: `python_qt_binding` 우선, `PyQt5` fallback
- ROS: `rclpy`
- goal 입력: `/goal_pose`
- 수동 조작: `/cmd_vel`

장점:

- Python 기반이라 현재 코드와 통합이 쉽다
- 현재 환경에서 바로 실행 가능했다
- RViz와 goal 입력 경로를 공유하므로 디버깅이 쉬웠다

주의:

- Qt 이벤트 루프와 ROS spin 루프를 분리해야 한다
- 실제 구현은 `QTimer` 기반 `spin_once()` 루프를 사용

### 대안

1. `rqt` 플러그인
   - ROS 친화적이지만 UI 자유도가 낮음

2. 웹 기반 GUI (`FastAPI` + `React` + rosbridge)
   - 확장성은 좋지만 현재 단계에서는 구현비용이 큼

3. Isaac Sim 내부 UI
   - 시뮬 전용으로 고립되기 쉬워 실로봇 공용 구조에 불리함

결론:
**실제 구현은 ROS 환경 적합성을 우선하여 `python_qt_binding/PyQt5` 기반으로 정리했다.**

---

## 명령 모델

자연어와 버튼 이벤트는 내부적으로 아래 공통 명령으로 변환한다.

```text
STOP
MOVE_RELATIVE(x_m, y_m)
ROTATE_RELATIVE(yaw_deg)
NAVIGATE_TO_POSE(x, y, yaw)
NAVIGATE_TO_WAYPOINT(name)
CANCEL_NAVIGATION
SAVE_CURRENT_POSE(name)
```

이렇게 표준화하면,

- 버튼 입력
- 텍스트 입력
- 나중의 음성 입력

모두 동일한 executor를 재사용할 수 있다.

---

## waypoint 전략

버튼과 자연어 모두 안정적으로 쓰려면 waypoint 체계가 먼저 필요하다.

### 저장 방식

파일: `config/waypoints.yaml`

초기 예시:

```yaml
waypoints:
  sample_name:
    frame_id: map
    x: 0.0
    y: 0.0
    yaw_deg: 0.0
```

### 필요한 기능

- waypoint 수동 등록
- 현재 위치를 waypoint로 저장
- GUI 목록에서 선택
- 이름 변경 / 삭제
- 필요 시 자연어 alias 매핑 확장

현재는 **초기 샘플 waypoint를 제거**하고, GUI에서 직접 저장한 waypoint만 사용하는 구조다.
즉 `config/waypoints.yaml` 초기 상태는 빈 목록이다.

초기 단계에서는 YAML 고정 파일로 충분하다.
DB는 아직 필요 없다.

---

## 자연어 처리 전략

### Phase 1: 규칙 기반 명령 해석

우선 아래 패턴부터 처리한다.

- 거리 이동
  - "앞으로 1m"
  - "뒤로 0.5미터"
  - "왼쪽으로 30cm"

- 회전
  - "왼쪽으로 90도 돌아"
  - "오른쪽으로 45도"

- 장소 이동
  - "L로 가"
  - "빨간색 박스 위로 이동"

- 제어
  - "멈춰"
  - "취소"

이 단계에서는 정규식 + alias 사전 + 단위 변환만으로 충분하다.

### Phase 2: 의미 해석 확장

아래 같은 표현은 후속 단계에서 지원한다.

- "문 앞까지 가"
- "책상 오른쪽으로 가"
- "지금 보는 방향으로 조금만 앞으로"

이 단계에서는 다음 중 하나를 붙일 수 있다.

1. 외부 LLM API
2. 로컬 소형 모델
3. rule-based + map semantics 혼합

단, 실제 실행 전에는 반드시 구조화 명령으로 validate해야 한다.
자연어 결과를 바로 `/cmd_vel`로 보내면 안 된다.

---

## 상대 이동 구현 방식

버튼 이동과 "앞으로 1미터" 같은 명령은 구현 방법이 2개 있다.

### 방법 A: `/cmd_vel` 직접 발행

장점:

- 구현이 빠름
- 짧은 움직임 테스트에 유리

단점:

- 장애물 회피 보장이 약함
- 거리 오차가 커질 수 있음

### 방법 B: 현재 pose 기준으로 목표 pose 계산 후 Nav2에 전달

예:

- 현재 pose `(x, y, yaw)` 조회
- "앞으로 1m" 입력
- `(x + cos(yaw), y + sin(yaw))` 계산
- Nav2 goal 발행

장점:

- Nav2 경로 계획과 충돌 회피 활용 가능
- 시뮬/실로봇 동작 일관성이 좋음

단점:

- pose 추정 품질에 의존
- 아주 짧은 미세 조작은 답답할 수 있음

결론:

- **대이동 명령은 Nav2 goal 방식**
- **정지 / 비상 / 미세조작은 `/cmd_vel` 방식**

으로 혼합 설계하는 것이 적절하다.

---

## Phase 계획

## Phase 0: 인터페이스 정의

목표:
- GUI가 다룰 명령 타입과 상태 모델을 먼저 고정

작업:
- [x] 내부 명령 enum / dataclass 정의
- [x] waypoint YAML 스키마 정의
- [x] GUI 상태 항목 정의
  - Nav2 연결 상태
  - 현재 pose
  - 현재 goal
  - goal 결과
- [x] 수동조작과 Nav2 goal 충돌 정책 정의

완료 기준:
- 버튼/텍스트 입력이 모두 공통 명령 모델로 떨어지는 설계 문서 확정

---

## Phase 1: 최소 GUI 구현

목표:
- 버튼 기반 제어와 상태 표시가 되는 데스크톱 GUI 구성

작업:
- [x] `go2_gui_controller` ROS2 Python 패키지 생성
- [x] Qt 기반 메인 윈도우 구성
  - 실제 구현은 `python_qt_binding` 우선, `PyQt5` fallback
- [x] 버튼 패널 추가
  - 전후좌우
  - 좌/우 회전
  - 정지
  - waypoint 선택 이동
- [x] 텍스트 입력창 / 실행 버튼 추가
- [x] 상태 패널 추가
  - current pose
  - nav status
  - last result

완료 기준:
- GUI가 실행되고 버튼 클릭 이벤트가 내부 명령으로 정상 변환됨

---

## Phase 2: Nav2 Goal 연동

목표:
- GUI에서 waypoint 이동과 goal 취소가 가능해짐

작업:
- [ ] `NavigateToPose` action client 구현
  - 현재는 `/goal_pose` 발행 방식으로 대체
- [~] goal 전송 / 피드백 / 결과 처리
- [x] 현재 goal 취소 버튼 추가
- [x] waypoint 목록 UI 추가
- [x] `config/waypoints.yaml` 로드 구현

완료 기준:
- RViz 없이 GUI에서 waypoint 선택 후 Nav2 이동 가능

---

## Phase 3: 수동조작 연동

목표:
- 짧은 구간 직접 제어와 정지가 가능함

작업:
- [x] `/cmd_vel` publisher 구현
- [x] 버튼 press/release 방식 또는 single-shot burst 방식 결정
- [x] 정지 버튼 최우선 처리
- [x] 수동조작 시작 시 Nav2 goal 자동 취소 구현

권장 정책:
- 전후좌우 버튼은 `press-and-hold`
- 회전 버튼도 `press-and-hold`
- 정지는 즉시 zero twist 3회 발행

완료 기준:
- GUI 버튼만으로 로봇을 저속 수동 조작 가능

---

## Phase 4: 규칙 기반 자연어 파서

목표:
- 자주 쓰는 한국어 명령을 구조화 명령으로 변환

작업:
- [x] 숫자/단위 파싱
  - m, meter, 미터, cm
- [x] 방향어 파싱
  - 앞, 뒤, 왼쪽, 오른쪽
- [x] 회전어 파싱
  - 돌기, 회전, 방향 전환
- [x] waypoint alias 사전 추가
- [x] 파싱 실패 메시지 정의

현재 구현 예:

```text
"앞으로 1미터 가" -> MOVE_RELATIVE(1.0, 0.0)
"오른쪽으로 90도 돌아" -> ROTATE_RELATIVE(-90)
"멈춰" -> STOP
"go to L" -> NAVIGATE_TO_WAYPOINT("L")
```

완료 기준:
- 지정한 핵심 패턴이 80% 이상 안정적으로 해석됨

---

## Phase 5: 상대 이동의 Nav2 변환

목표:
- 자연어 상대 이동도 Nav2 기반 안전 이동으로 처리

작업:
- [x] 현재 pose 조회 구현 (`tf2` 또는 `/odom`)
- [x] `MOVE_RELATIVE`, `ROTATE_RELATIVE`를 목표 pose로 변환
- [x] 변환된 pose를 Nav2 goal로 전송
  - 현재 구현은 RViz 호환 `/goal_pose`
- [ ] 도달 허용 오차 튜닝

완료 기준:
- "앞으로 1미터"가 단순 `/cmd_vel`이 아니라 Nav2 goal로 실행됨

---

## Phase 6: 검증 및 실로봇 전환

목표:
- 시뮬과 실로봇에서 동일한 GUI 운용 절차 확보

작업:
- [x] Isaac Sim + Nav2에서 GUI 테스트
- [ ] waypoint 이동 성공률 측정
- [ ] 자연어 명령 성공/실패 케이스 로그화
- [ ] 실로봇 launch와 토픽명 차이 점검
- [ ] 비상정지 절차 문서화

완료 기준:
- 시뮬과 실로봇 모두에서 GUI로 waypoint 이동 + 정지 + 기본 자연어 이동 가능

---

## 07단계 이후 확장 계획: 단일 창 통합 GUI

현재 07단계 GUI는 **controller + waypoint + 기본 상태 표시** 중심이다.

하지만 실제 운용 관점에서는

- 터미널 여러 개
- RViz / rqt_plot / 별도 시각화 창 여러 개
- GUI controller 별도 창

처럼 분리해서 쓰는 방식보다,
**하나의 메인 GUI 창 안에 운용 기능과 주요 모니터링 기능을 계속 확장하는 구조**가 더 적합하다.

즉 다음 단계의 방향은
`외부 툴 의존을 완전히 없애는 것`이 아니라,
**자주 보는 핵심 기능을 현재 GUI 안으로 점진적으로 흡수하는 것**이다.

### 목표

- 하나의 메인 창에서 운용에 필요한 핵심 기능을 대부분 처리
- 수동조작 / waypoint / 자연어 / 상태 / telemetry / 그래프를 한 앱 안에서 제공
- 디버그용 외부 툴은 필요 시에만 보조적으로 사용

### 설계 원칙

1. GUI는 여전히 **운용 중심 UI**여야 한다.
   - 모든 ROS topic을 범용적으로 탐색하는 툴을 처음부터 만들지 않는다.
   - Go2 운영에 필요한 항목부터 고정형 패널로 넣는다.

2. 한 창으로 통합하되, 코드 구조는 모듈화한다.
   - 메인 윈도우는 탭 또는 dock 기반으로 확장
   - control / monitor / charts / logs 성격을 분리

3. 상태 조회와 시각화는 별도 bridge 계층으로 분리한다.
   - 현재 `state_bridge.py`는 pose 중심이다.
   - 향후 telemetry 전용 bridge를 추가해 시계열 데이터를 수집한다.

4. 차트는 `범용 topic plotter`보다 `운용용 고정 차트`를 우선한다.
   - 예: joint position / joint velocity / cmd_vel / odom speed

### 권장 UI 구조

```text
Go2 Integrated GUI
  |- Control Tab
  |    |- manual control
  |    |- waypoint
  |    |- text / voice command
  |
  |- Monitor Tab
  |    |- current pose
  |    |- nav status
  |    |- battery / mode / connection
  |    |- topic heartbeat
  |
  |- Charts Tab
  |    |- joint position
  |    |- joint velocity
  |    |- cmd_vel / odom / imu
  |
  |- Logs Tab
       |- command log
       |- warning / error / event history
```

### 우선순위

#### Phase 7: GUI 구조 리팩터링

목표:
- 현재 단일 화면 UI를 확장 가능한 메인 창 구조로 정리

작업:
- [ ] 메인 창을 탭 기반으로 재구성
- [ ] 기존 제어 UI를 `Control` 탭으로 분리
- [ ] 상태 표시와 로그를 `Monitor` / `Logs` 성격으로 정리
- [ ] 기능별 widget / panel 클래스로 파일 분리

완료 기준:
- 새 기능을 추가해도 `gui_app.py` 하나에 로직이 과도하게 몰리지 않음

#### Phase 8: Telemetry 수집 계층 추가

목표:
- 그래프와 상태 패널이 공통으로 사용할 telemetry 데이터를 수집

작업:
- [ ] `telemetry_bridge.py` 추가
- [ ] `/joint_states` subscribe
- [ ] `/odom` subscribe
- [ ] `/cmd_vel` subscribe
- [ ] 필요 시 `/imu/data` subscribe
- [ ] 최근 N초 ring buffer 저장 구조 추가
- [ ] 토픽 수신 여부 / 최근 수신 시각 / 간단한 heartbeat 상태 표시

완료 기준:
- GUI 내부에서 실시간 수치와 최근 시계열 데이터를 재사용 가능

#### Phase 9: 내장 Charts / Monitor 구현

목표:
- 외부 plot 툴 없이도 주요 로봇 상태를 메인 GUI 안에서 바로 확인

작업:
- [ ] `Charts` 탭 추가
- [ ] joint position 그래프
- [ ] joint velocity 그래프
- [ ] cmd_vel / odom 속도 그래프
- [ ] topic summary 패널
- [ ] 그래프 표시 대상 joint 선택 UI
- [ ] 그래프 라이브러리 의존성 정리 (`pyqtgraph` 우선 검토)

완료 기준:
- 사용자가 GUI 한 창 안에서 Go2 주요 상태와 관절 변화를 바로 확인 가능

#### Phase 10: 운용 편의 기능 확장

목표:
- 단순 controller를 넘어서 실제 operator console 수준으로 발전

작업 후보:
- [ ] 저장 waypoint 그룹 관리
- [ ] 최근 명령 / 최근 실패 이력 패널
- [ ] 비상정지 / recover / reset 버튼 정리
- [ ] 배터리 / 네트워크 / 센서 연결 상태 표시
- [ ] 실로봇 전용 모드와 시뮬 모드 UI 차등 표시
- [ ] 주요 launch 상태 요약 또는 내부 프로세스 상태 표시

완료 기준:
- 일상 운용 시 별도 터미널/창 의존도가 크게 줄어듦

### 범위 주의

이 확장 계획은
`ROS 디버그 도구 전체를 GUI 안에 재구현`하는 것이 아니다.

우선순위는 아래처럼 둔다.

1. 운영자가 매번 보는 기능은 GUI 안으로 통합
2. 가끔 보는 정밀 디버깅 기능은 외부 툴 유지

즉 `rqt_plot`, `RViz2`, `Foxglove`, `PlotJuggler` 같은 도구를 완전히 대체하려 하기보다,
**운용 빈도가 높은 화면부터 현재 GUI에 흡수**하는 전략을 권장한다.

---

## ROS 인터페이스 초안

### Subscribe

- `/odom`
- `/tf`
- `/tf_static`

### Publish

- `/cmd_vel`
- `/goal_pose`

### Service Client

- `/navigate_to_pose/_action/cancel_goal`

### 현재 결론

- goal 입력은 `/goal_pose`
- 직접 action client보다 RViz와 동일한 goal 경로를 우선 사용
- cancel만 action cancel service 사용

---

## 런치 전략

### 신규 launch 제안

파일:
- `src/go2_gui_controller/launch/go2_gui_controller.launch.py`

역할:
- GUI controller 실행
- waypoint config 경로 전달
- sim / real 모드 인자 제공

예시 구조:

```text
ros2 launch go2_gui_controller go2_gui_controller.launch.py mode:=sim
ros2 launch go2_gui_controller go2_gui_controller.launch.py mode:=real
```

현재 저장소가 launch 파일을 루트 `launch/`에 두고 있으므로,
초기 통합은 아래 둘 중 하나로 정리할 수 있다.

1. 새 ROS2 패키지 내부 launch로 분리
2. 현재 저장소 `launch/`에 브리지 launch만 두고 실제 노드는 패키지에서 실행

권장:
**노드는 패키지 내부, 프로젝트 루트 launch는 통합 진입점 역할**

---

## 리스크와 대응

### 1. Qt 이벤트 루프와 ROS spin 충돌

대응:
- GUI 메인 스레드 + ROS worker thread 분리
- 또는 Qt timer에서 `spin_once()` 호출

### 2. 자연어 해석의 모호성

대응:
- 처음엔 규칙 기반 범위를 명확히 제한
- 해석 결과를 실행 전 GUI에 한 줄로 표시
- 모호하면 실행하지 않고 재입력 요청

### 3. 상대 이동 시 pose 오차 누적

대응:
- 큰 이동은 Nav2 pose goal 사용
- 아주 짧은 미세 조정만 `/cmd_vel` 허용

### 4. 실로봇 안전성

대응:
- GUI 정지 버튼 최상단 고정
- goal 취소와 zero twist 즉시 발행
- 실로봇에서는 속도 상한을 별도 보수적으로 설정

### 5. `map` pose를 ground truth처럼 오해할 위험

현재 시뮬 구조는 아래와 같다.

```text
map --(RTAB-Map localization)--> odom --(Isaac ground truth)--> base_link
```

즉:

- `odom -> base_link` 는 Isaac Sim ground-truth
- `map -> odom` 는 RTAB-Map 추정 결과

따라서 GUI에 `Pose: map: ...` 로 보이는 값은 ground-truth가 아니라
**RTAB-Map localization 결과가 섞인 global pose**다.

맵이 단순하거나 반복 구조가 많으면,
RTAB-Map이 현재 위치 대신 다른 비슷한 위치를 더 맞는 장소로 오인하여
`map -> odom`이 갑자기 튈 수 있다.

즉 "odom이 누적 오차가 생긴 것"이 아니라,
"global localization이 잘못 맞아서 map 기준 pose가 흔들리는 것"일 가능성이 높다.

---


## 현재 지원 명령

### 버튼 조종

- `Forward`
- `Back`
- `Left`
- `Right`
- `Turn Left`
- `Turn Right`
- `Stop`
- `Go To Selected Waypoint`
- `Delete Waypoint`
- `Save Current Pose`
- `Rename Waypoint`
- `Cancel Goal`

### 텍스트 / 자연어 명령

현재 구현은 자유대화형 자연어가 아니라,
**규칙 기반 명령형 입력**이다.

지원 예:

- `앞으로 1미터 가`
- `뒤로 0.5미터 가`
- `왼쪽으로 50cm 가`
- `오른쪽으로 1m 가`
- `앞으로 조금 가`
- `왼쪽으로 90도 돌아`
- `오른쪽으로 45도 회전`
- `왼쪽으로 반바퀴 돌아`
- `멈춰`
- `멈춰줘`
- `정지`
- `취소`
- `그만`
- `forward 1m`
- `left 0.5m`
- `turn right 90`
- `go to L`

waypoint 이름도 직접 입력할 수 있다.

예:

- `L`
- `빨간색 박스 위`
- `파란색 박스 위`

### 아직 지원하지 않는 자연어

- `문 앞까지 가`
- `책상 오른쪽으로 가`
- `장애물 피해 저쪽으로 가`
- `앞으로 천천히 가`

즉 현재는:

- 방향 + 거리
- 방향 + 각도
- 정지 / 취소
- waypoint 이름 이동

위주로 지원한다.

---

## 마지막 논의 정리

### Q1. ground-truth odom인데 왜 움직이다 보면 위치가 틀어져 보이나?

답:

- `odom -> base_link`는 ground-truth라도
- GUI에 보이는 `map` 기준 pose는 `RTAB-Map localization`이 섞인 값이다
- 따라서 단순한 맵에서는 RTAB-Map이 다른 비슷한 위치를 더 맞다고 오인해서
  `map -> odom`이 크게 튈 수 있다

즉 "odom 누적 오차"가 아니라
"map localization 오인식"일 가능성이 크다.

### Q2. waypoint로 이동했더니 x,y는 맞는데 yaw는 다르게 끝난다

답:

- waypoint goal에 저장 yaw를 같이 보내더라도
- Nav2의 goal checker / controller / MPPI 거동 때문에
  최종 자세가 완전히 저장 당시와 같지 않을 수 있다
- 이것은 GUI의 기록 문제라기보다 Nav2 goal 처리의 특성에 더 가깝다

### Q3. 왜 RViz는 되는데 GUI는 안 되는 구간이 있었나?

답:

- direct action client / `nav2_simple_commander` 경로보다
- 현재 프로젝트에서는 RViz와 같은 `/goal_pose` 경로가 더 안정적이었다
- 그래서 최종 구현도 RViz 호환 방식으로 정리했다

---

## 권장 구현 순서

1. GUI 패키지 생성
2. 버튼 UI + 상태 패널 구현
3. Nav2 goal 연동
4. waypoint YAML 로딩
5. `/cmd_vel` 수동 조작
6. 규칙 기반 자연어 파서
7. 상대 이동을 pose goal로 변환
8. 시뮬 검증 후 실로봇 이식

이 순서를 권장하는 이유는,
자연어보다 먼저 **버튼 + waypoint + Nav2 연동**을 완성해야
문제 구간을 GUI, parser, Nav2 중 어디서 찾을지 명확해지기 때문이다.

---

## 최종 완료 조건

아래 시나리오가 모두 가능하면 07단계 완료로 본다.

1. GUI에서 저장된 waypoint를 선택해 Nav2 goal 이동 가능
2. GUI에서 정지 버튼으로 현재 goal 취소 + 로봇 정지 가능
3. "앞으로 1미터 가", "왼쪽으로 90도 돌아", "L로 가" 명령 실행 가능
4. 현재 위치와 마지막 명령 결과를 GUI에서 확인 가능
5. RViz 없이도 동일한 조작 흐름이 성립함

---
## 현재 상태 요약

07단계 GUI controller는 현재 다음 수준까지 구현되었다.

- `go2_gui_controller` 패키지 생성 완료
- GUI 수동 조작 완료
- waypoint 저장 / 삭제 / 이름 변경 완료
- waypoint goal 발행 완료
- 규칙 기반 자연어 입력 완료
- RViz 호환 `/goal_pose` 방식 적용 완료
- Nav2 파라미터 체인 (`cmd_vel_nav -> velocity_smoother`) 문제 해결 완료

남은 과제는 주로 아래다.

- 자연어 범위 확대
- 도움말/사용 가능 명령 표시 UI 추가
- map pose / odom pose 동시 표시 여부 검토
- Nav2 최종 자세(yaw) 정렬 품질 추가 검증
- 단일 창 통합 GUI 구조로 리팩터링
- telemetry bridge 및 내장 charts / monitor 추가
