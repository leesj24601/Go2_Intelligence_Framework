# Implementation Plan: HTML 웹 기반 GUI 컨트롤러 (Phase 7)

> 업데이트: 2026-04-08

## 배경 및 목적

기존 PyQt5 기반 GUI(`gui_app.py`)를 **웹 브라우저에서 동작하는 HTML/JS UI로 대체**한다.

- Claude가 브라우저 UI를 직접 보면서 협업 가능 (스크린샷 기반 디버깅)
- Qt 설치 의존성 없이 PC 브라우저에서 접근 가능
- Qt GUI는 당분간 병행 유지 (`gui_app.py` 삭제하지 않음)

## 이번 구현 범위 확정 사항

- 이번 작업은 **Phase 7 HTML 웹 GUI 구현만** 대상으로 한다.
- `sim` / `real` **두 런타임 모드 모두 지원**한다.
- 모드 선택은 **브라우저 UI가 아니라 실행 인자**(`mode:=sim|real`)로 결정한다.
- 웹 서버는 **PC 로컬 접속 전용**으로 운영하며 기본 바인딩은 `127.0.0.1:8080`으로 둔다.
- 음성 명령(STT)은 **이번 범위에 포함**한다.
- 웨이포인트 파일은 기존 Qt GUI와 **동일한 `config/waypoints.yaml`을 공유**한다.
- Runtime 제어는 기존 Qt GUI와 동일하게 **`SLAM` / `Navigation` 상호 배타 정책**을 유지한다.
- 수동 조작 속도 상수는 기존 Qt GUI와 **동일한 값**을 유지한다.
- 웹 UI는 **로그인/권한 관리 없는 단일 운영자용 로컬 제어 페이지**로 구현한다.
- Python 의존성 추가 시 `requirements.txt`뿐 아니라 **ROS 패키지 메타데이터도 함께 정리**한다.

**이번 범위에서 제외 (추후 확장):**
- Charts 탭 (joint 상태, 속도 비교 그래프)
- 맵 시각화 + 클릭으로 목표 지점 설정
- 카메라 피드 (`/camera/color/image_raw`)

---

## 아키텍처

```
브라우저 (index.html)
    ↕  WebSocket ws://localhost:8080/ws   ← 상태 실시간 수신 (50ms 주기)
    ↕  HTTP POST http://localhost:8080/cmd/...  ← 명령 전송
FastAPI 서버 (web_app.py)
    ↕  rclpy (시스템 Python, 별도 스레드에서 spin)
ROS2 bridge 클래스들 (기존 코드 그대로 재사용)
```

---

## 파일 구조

### 신규 생성

```
src/go2_gui_controller/go2_gui_controller/
├── web_app.py              ← FastAPI 서버 + rclpy 노드
├── web/
│   └── index.html          ← 순수 HTML/CSS/JS 단일 파일
└── web_launch_manager.py   ← LaunchManager 대체 (QProcess 제거, subprocess 사용)
```

### 수정

```
src/go2_gui_controller/
├── setup.py                ← web_controller entry point 추가
└── launch/
    └── go2_web_controller.launch.py  ← 신규 launch 파일

requirements.txt (루트)     ← fastapi, uvicorn 추가
```

### 변경 없음 (그대로 재사용)

```
go2_gui_controller/
├── state_bridge.py
├── navigator_bridge.py
├── manual_control.py
├── text_command_parser.py
├── voice_command_listener.py
├── waypoint_registry.py
├── commands.py
└── gui_app.py              ← Qt GUI 그대로 유지
```

> **TelemetryBridge**: Charts 탭이 이번 범위에서 제외되므로 `web_app.py`에서 초기화하지 않는다.
> 추후 Charts 탭 구현 시 추가한다.

---

## 주요 설계 결정

### RuntimeMode 처리

웹 버전은 Qt GUI처럼 시작 후 팝업으로 모드를 고르지 않는다. 대신 **프로세스 시작 시 launch/CLI 인자에서 런타임 모드를 고정**한다.

- 이유: 브리지 초기화 시점부터 `odom_topic` / `use_sim_time`을 올바르게 설정할 수 있어 구현이 단순하고 안정적이다.
- 추후 필요하면 브라우저 첫 화면에서 모드를 고르는 UX를 별도 확장할 수 있지만, 이번 1차 범위에서는 제외한다.

Qt GUI는 시작 시 Sim/Real 모드를 선택하며 각각 다른 설정을 가진다:

```python
# gui_app.py에 정의된 기존 상수
SIM_MODE  → odom_topic="/odom",               use_sim_time=True
REAL_MODE → odom_topic="/utlidar/robot_odom", use_sim_time=False
```

`StateBridge`는 `odom_topic` 파라미터를 받아 구독 토픽을 결정한다:
```python
StateBridge(node, odom_topic="/utlidar/robot_odom")  # real
StateBridge(node, odom_topic="/odom")                 # sim
```

**웹 버전 처리 방식**: launch 파일 argument로 `mode:=sim` 또는 `mode:=real`을 받아 ROS2 파라미터로 전달한다.

```bash
# 시뮬레이션
ros2 run go2_gui_controller web_controller --ros-args -p mode:=sim

# 실로봇
ros2 run go2_gui_controller web_controller --ros-args -p mode:=real
```

`web_app.py`에서 처리:
```python
RUNTIME_MODES = {
    "sim":  {"odom_topic": "/odom",               "use_sim_time": True},
    "real": {"odom_topic": "/utlidar/robot_odom",  "use_sim_time": False},
}

class WebControllerNode(Node):
    def __init__(self):
        super().__init__("go2_web_controller")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("waypoint_file", "")
        mode_key = self.get_parameter("mode").value
        mode = RUNTIME_MODES.get(mode_key, RUNTIME_MODES["sim"])
        odom_topic = mode["odom_topic"]
        # use_sim_time은 ROS2 내장 파라미터이므로 별도 처리
        self.set_parameters([Parameter("use_sim_time", value=mode["use_sim_time"])])
        ...
        self.state_bridge = StateBridge(self, odom_topic=odom_topic)
```

### LaunchManager 교체 필요

기존 `launch_manager.py`는 `QProcess` / `QTimer`에 의존한다:

```python
from python_qt_binding.QtCore import QProcess, QTimer  # Qt 의존
```

웹 버전에서는 Qt가 없으므로 `web_launch_manager.py`를 별도로 만들어 `subprocess.Popen` 기반으로 교체한다. **인터페이스(메서드명, 시그니처, 반환값)는 기존 `LaunchManager`와 동일하게 유지**한다.

또한 기존 동작과 동일하게 다음 운영 정책을 유지한다.

- `slam` 시작 요청 시 `navigation`이 실행 중이면 먼저 중지한다.
- `navigation` 시작 요청 시 `slam`이 실행 중이면 먼저 중지한다.
- 즉 두 스택은 동시에 활성화하지 않는다.

```python
# web_launch_manager.py
import subprocess
import threading

class WebLaunchManager:
    # 기존 LaunchManager와 동일한 퍼블릭 인터페이스:
    # start(key: str) -> tuple[bool, str]
    # stop(key: str) -> tuple[bool, str]
    # open_rviz() -> tuple[bool, str]
    # close_rviz() -> tuple[bool, str]
    # shutdown() -> None
    # status_text(key: str) -> str          # "stopped" | "starting" | "running" | "stopping" | "crashed"
    # rviz_status_text() -> str
    # is_process_running(key: str) -> bool
    # available: bool (property)
    #
    # QProcess  → subprocess.Popen
    # QTimer.singleShot(3000, fn) → threading.Timer(3.0, fn).start()
```

### 스레드 안전성

rclpy는 별도 스레드에서 spin하며 bridge 클래스들의 상태를 업데이트한다.
FastAPI asyncio 루프(메인 스레드)에서 이 상태를 읽을 때 race condition이 발생할 수 있으므로
**`threading.Lock`으로 상태 읽기/쓰기를 보호**한다.

```python
class WebControllerNode(Node):
    def __init__(self):
        ...
        self._lock = threading.Lock()
        self._logs: list[str] = []
        self._stack_status: dict[str, str] = {"slam": "stopped", "navigation": "stopped", "rviz": "stopped"}
        self._pending_log: str | None = None

    def _log_callback(self, message: str) -> None:
        with self._lock:
            self._logs.append(message)
            if len(self._logs) > 200:
                self._logs.pop(0)
            self._pending_log = message

    def get_state_snapshot(self) -> dict:
        """asyncio 루프에서 호출 — lock으로 보호된 스냅샷 반환"""
        with self._lock:
            state = self.state_bridge.state
            import math
            return {
                "pose": {
                    "x": state.x,
                    "y": state.y,
                    "yaw_deg": math.degrees(state.yaw_rad),
                    "frame_id": state.frame_id,
                },
                "nav_status": state.nav_status,
                "last_result": state.last_result,
                "localized": state.localization_initialized,
                "waypoints": [w.name for w in self.waypoint_registry.list_waypoints()],
                "stacks": dict(self._stack_status),
                "log": self._pending_log,
            }

    def clear_pending_log(self) -> None:
        with self._lock:
            self._pending_log = None
```

---

## web_app.py 상세

### 클래스 구조

```python
class WebControllerNode(Node):
    def __init__(self):
        super().__init__("go2_web_controller")
        # 파라미터 선언
        self.declare_parameter("mode", "sim")
        self.declare_parameter("waypoint_file", "")
        mode_key = self.get_parameter("mode").value
        mode = RUNTIME_MODES.get(mode_key, RUNTIME_MODES["sim"])
        odom_topic = mode["odom_topic"]

        # waypoint 파일 경로 결정
        waypoint_file_param = self.get_parameter("waypoint_file").value
        if waypoint_file_param:
            waypoint_file = Path(waypoint_file_param)
        else:
            # ament_index에서 패키지 share 경로 찾기
            from ament_index_python.packages import get_package_share_directory
            waypoint_file = Path(get_package_share_directory("go2_gui_controller")) / "config" / "waypoints.yaml"

        # bridge 초기화 (TelemetryBridge는 Charts 탭 구현 시 추가)
        self._lock = threading.Lock()
        self.waypoint_registry = WaypointRegistry(waypoint_file)
        self.state_bridge = StateBridge(self, odom_topic=odom_topic)
        self.navigator_bridge = NavigatorBridge(self, self.state_bridge, self.waypoint_registry)
        self.manual_bridge = ManualControlBridge(self)
        self.text_parser = TextCommandParser(self.waypoint_registry)
        self.voice_listener = VoiceCommandListener()
        self.launch_manager = WebLaunchManager(
            runtime_mode_key=mode_key,
            log_callback=self._log_callback,
            status_callback=self._status_callback,
        )
        self._logs: list[str] = []
        self._stack_status = {"slam": "stopped", "navigation": "stopped", "rviz": "stopped"}
        self._pending_log: str | None = None
```

### 텍스트 명령 dispatch 로직

`/cmd/text` 엔드포인트에서 parse 후 반드시 아래 로직으로 실행해야 한다:

```python
def execute_parsed_command(node: WebControllerNode, cmd: ParsedCommand) -> str:
    if cmd.command_type == CommandType.STOP:
        node.manual_bridge.stop()
        return "stopped"
    elif cmd.command_type == CommandType.CANCEL_NAVIGATION:
        node.navigator_bridge.cancel()
        return "navigation cancelled"
    elif cmd.command_type == CommandType.MOVE_RELATIVE:
        node.navigator_bridge.go_to_relative_pose(cmd.x_m, cmd.y_m, cmd.yaw_deg)
        return f"moving relative x={cmd.x_m}m y={cmd.y_m}m"
    elif cmd.command_type == CommandType.ROTATE_RELATIVE:
        node.navigator_bridge.go_to_relative_pose(0.0, 0.0, cmd.yaw_deg)
        return f"rotating {cmd.yaw_deg}deg"
    elif cmd.command_type == CommandType.NAVIGATE_TO_WAYPOINT:
        node.navigator_bridge.go_to_waypoint(cmd.waypoint_name)
        return f"navigating to {cmd.waypoint_name}"
    return "unknown command"
```

### HTTP 엔드포인트

| 메서드 | 경로 | 요청 바디 | 동작 |
|--------|------|---------|------|
| GET | `/` | — | `index.html` 반환 |
| WS | `/ws` | — | 상태 브로드캐스트 (50ms) |
| POST | `/cmd/manual` | `{"direction":"forward"}` | `ManualControlBridge.send_velocity()` |
| POST | `/cmd/stop` | — | `ManualControlBridge.stop()` |
| POST | `/cmd/navigate` | `{"waypoint_name":"home"}` | `NavigatorBridge.go_to_waypoint()` |
| POST | `/cmd/cancel` | — | `NavigatorBridge.cancel()` |
| POST | `/cmd/text` | `{"text":"앞으로 1미터"}` | `TextCommandParser.parse()` → `execute_parsed_command()` |
| POST | `/cmd/voice` | — | `VoiceCommandListener.listen_once()` → `execute_parsed_command()` |
| POST | `/cmd/waypoint/save` | `{"name":"desk"}` | 현재 pose로 `WaypointRegistry.save_waypoint(name, frame_id, x, y, yaw_deg)` |
| POST | `/cmd/waypoint/delete` | `{"name":"desk"}` | `WaypointRegistry.delete_waypoint(name)` |
| POST | `/cmd/waypoint/rename` | `{"old_name":"desk","new_name":"office"}` | `WaypointRegistry.rename_waypoint(old_name, new_name)` |
| POST | `/stack/start` | `{"target":"slam"}` | `WebLaunchManager.start(target)` |
| POST | `/stack/stop` | `{"target":"slam"}` | `WebLaunchManager.stop(target)` |
| POST | `/stack/rviz/open` | — | `WebLaunchManager.open_rviz()` |
| POST | `/stack/rviz/close` | — | `WebLaunchManager.close_rviz()` |

**waypoint 저장 시 현재 pose 추출:**
```python
state = node.state_bridge.state
import math
node.waypoint_registry.save_waypoint(
    name=name,
    frame_id=state.frame_id,
    x=state.x,
    y=state.y,
    yaw_deg=math.degrees(state.yaw_rad),
)
```

### WebSocket 상태 메시지 포맷

50ms마다 브로드캐스트:

```json
{
  "pose": {
    "x": 1.23,
    "y": -0.45,
    "yaw_deg": 90.0,
    "frame_id": "map"
  },
  "nav_status": "navigating:home (1.20m)",
  "last_result": "succeeded:home",
  "localized": true,
  "waypoints": ["home", "desk", "entrance"],
  "stacks": {
    "slam": "stopped",
    "navigation": "running",
    "rviz": "stopped"
  },
  "log": "[Control] 웨이포인트 home으로 이동 명령"
}
```

`log` 필드: 새 로그가 있을 때만 포함, 없으면 `null`. 브로드캐스트 후 `clear_pending_log()` 호출.

### rclpy + asyncio 통합

```python
# rclpy spin 전용 스레드 (daemon=True로 메인 종료 시 자동 종료)
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

# uvicorn으로 FastAPI 실행 (메인 스레드 asyncio 루프)
uvicorn.run(app, host="127.0.0.1", port=8080)
```

---

## index.html 상세

순수 HTML + CSS + Vanilla JS. 외부 CDN 없음.

### 탭 구성

**Control 탭**
```
┌─────────────────────────────────────────┐
│ Pose: map: x=1.23, y=-0.45, yaw=90.0°  │
│ Nav:  navigating:home (1.20m)           │
│ Last: succeeded:home                    │
├──────────────────┬──────────────────────┤
│ Manual Control   │ Waypoints            │
│                  │ [home           ▼]   │
│   [  Forward  ]  │ [Go] [Cancel]        │
│ [Left][Stop][Rt] │ Save as:             │
│   [ Backward  ]  │ [__________][Save]   │
│ [Turn L][Turn R] │ Rename to:           │
│                  │ [__________][Rename] │
│                  │ [Delete Selected]    │
├──────────────────┴──────────────────────┤
│ Text Command                            │
│ [앞으로 1미터________________] [Run]    │
│ Voice: [Listen]  status: ready          │
└─────────────────────────────────────────┘
```

**Runtime 탭**
```
┌─────────────────────────────────────┐
│ SLAM        [Start] [Stop]  stopped  │
│ Navigation  [Start] [Stop]  running  │
│ RViz        [Open]  [Close] stopped  │
└─────────────────────────────────────┘
```

**Logs 탭**
```
┌─────────────────────────────────────┐
│ [Clear]                              │
│ 14:23:01 [Control] home으로 이동     │
│ 14:23:05 [Nav] succeeded:home        │
│ ...                                  │
└─────────────────────────────────────┘
```

### 수동 버튼 동작

버튼을 누르는 동안 50ms마다 POST 반복:

```javascript
let manualInterval = null;

function bindHoldButton(btn, direction) {
    const start = () => {
        fetch('/cmd/manual', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({direction})
        });
        manualInterval = setInterval(() => {
            fetch('/cmd/manual', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction})
            });
        }, 50);
    };
    const stop = () => {
        clearInterval(manualInterval);
        manualInterval = null;
        fetch('/cmd/stop', {method: 'POST'});
    };
    btn.addEventListener('mousedown', start);
    btn.addEventListener('mouseup', stop);
    btn.addEventListener('mouseleave', stop);  // 버튼 밖으로 나갔을 때도 정지
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); start(); });
    btn.addEventListener('touchend', stop);
}
```

### 속도 상수 (gui_app.py 기준 동일)

```javascript
const SPEED = {
    forward:  { lx: 1.0,  ly: 0.0,  az: 0.0 },
    backward: { lx: -1.0, ly: 0.0,  az: 0.0 },
    left:     { lx: 0.0,  ly: 0.6,  az: 0.0 },
    right:    { lx: 0.0,  ly: -0.6, az: 0.0 },
    turn_l:   { lx: 0.0,  ly: 0.0,  az: 1.0 },
    turn_r:   { lx: 0.0,  ly: 0.0,  az: -1.0 },
};
```

백엔드에서 `direction` 문자열을 받아 위 테이블로 `send_velocity()` 호출.

### WebSocket 재연결 로직

서버 재시작 시 자동 재연결:

```javascript
function connectWebSocket() {
    const ws = new WebSocket('ws://localhost:8080/ws');

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        updateUI(data);
    };

    ws.onclose = () => {
        // 3초 후 재연결 시도
        setTimeout(connectWebSocket, 3000);
    };

    ws.onerror = () => {
        ws.close();
    };
}

connectWebSocket();
```

### 운영 가정

이번 웹 UI는 원격 다중 사용자 콘솔이 아니라 **로컬 PC에서 운영자가 직접 여는 단일 페이지**를 전제로 한다.

- 인증/로그인 기능은 넣지 않는다.
- 권한 분리나 사용자 세션 개념은 넣지 않는다.
- 접속 대상은 기본적으로 서버를 실행한 동일 PC의 브라우저다.

---

## 구현 순서

1. **`web_launch_manager.py`** — `subprocess.Popen` 기반, `LaunchManager`와 동일한 인터페이스
2. **`web_app.py`** — `WebControllerNode` (RuntimeMode 처리, lock, state snapshot 포함), FastAPI 앱, 모든 엔드포인트
3. **`web/index.html`** — Control 탭 (버튼, waypoint rename 포함, 텍스트/음성, 상태 표시, WebSocket 재연결)
4. **`web/index.html`** — Runtime 탭 + Logs 탭 추가
5. **`setup.py`** — `web_controller` entry point 추가
6. **`launch/go2_web_controller.launch.py`** — launch 파일 신규 생성
7. **`requirements.txt`** — `fastapi`, `uvicorn[standard]` 추가

---

## 의존성

```bash
pip install fastapi "uvicorn[standard]"
```

`requirements.txt`에 추가:
```
fastapi
uvicorn[standard]
```

추가로 배포/재설치를 고려해 `package.xml` 등 ROS 패키지 메타데이터에도 필요한 의존성을 반영한다.

---

## 실행 방법

```bash
# 빌드 (go2_gui_controller_ws 기준)
colcon build --packages-select go2_gui_controller
source install/setup.bash

# 실행 (시뮬레이션)
ros2 run go2_gui_controller web_controller --ros-args -p mode:=sim

# 실행 (실로봇)
ros2 run go2_gui_controller web_controller --ros-args -p mode:=real

# 브라우저
http://127.0.0.1:8080
```

launch 파일로 실행:
```bash
ros2 launch go2_gui_controller go2_web_controller.launch.py mode:=sim
```

---

## 추후 확장 항목

| 항목 | 방법 |
|------|------|
| Charts 탭 | `TelemetryBridge` 초기화 추가, WebSocket으로 joint_states/cmd_vel/odom 전송 → Chart.js 렌더링 |
| 맵 시각화 | `/map` (OccupancyGrid) WebSocket 수신 → `<canvas>` 렌더링 |
| 클릭 목표 지점 | 맵 canvas 클릭 → 픽셀 → map 좌표 변환 → `POST /cmd/navigate_pose` |
| 카메라 피드 | `/camera/color/compressed` → MJPEG 스트림 또는 base64 WebSocket |
