# Implementation Plan: HTML 웹 GUI v2 — Charts 탭 + 클린 화이트 디자인 (Phase 7.1)

> 업데이트: 2026-04-09

## 배경 및 목적

Phase 7에서 완성한 HTML 웹 GUI(`web_app.py` + `index.html`)에 두 가지 기능을 추가한다.

1. **Charts 탭**: Go2 관절값(position/velocity) 및 cmd vs odom 속도 비교 그래프 (진단/디버깅용)
2. **UI 디자인 전면 개편**: 현재 아이보리/포레스트그린 팔레트 → 클린 화이트

현재 `TelemetryBridge`는 구현 완료되어 있으나 `web_app.py`에서 초기화되지 않은 상태. 이번 작업에서 연동 완료.

---

## 이번 구현 범위

- `TelemetryBridge` 초기화 및 REST 엔드포인트 3개 추가
- Charts 탭: 관절 position/velocity 그래프 (관절 선택 가능), cmd vs odom 속도 비교 그래프
- UI 전체 색상 토큰 교체 (클린 화이트)
- Chart.js 4.x 인라인 번들 (CDN 없음, 단일 HTML 파일 유지)
- Charts 탭 활성 시에만 500ms 폴링 (탭 미사용 시 오버헤드 없음)

**이번 범위에서 제외 (추후 확장):**
- 맵 시각화 + 클릭 목표 지점
- 카메라 피드
- 다크 모드 토글

---

## 수정 파일

| 파일 | 작업 |
|------|------|
| `src/go2_gui_controller/go2_gui_controller/web_app.py` | TelemetryBridge 초기화 + telemetry REST 엔드포인트 3개 추가 |
| `src/go2_gui_controller/go2_gui_controller/web/index.html` | 클린 화이트 디자인 전면 교체 + Charts 탭 추가 |

---

## 1. 백엔드 변경 (`web_app.py`)

### TelemetryBridge 초기화

`WebControllerNode.__init__` 의 bridge 초기화 블록에 추가 (`StateBridge` 초기화 바로 아래 권장):

```python
from .telemetry_bridge import TelemetryBridge
self.telemetry_bridge = TelemetryBridge(self, odom_topic=self.runtime_mode["odom_topic"])
```

> `TelemetryBridge.__init__(self, node, odom_topic)` — `telemetry_bridge.py` 참조  
> ⚠️ `__init__` 안에 `odom_topic` 지역변수 없음. 반드시 `self.runtime_mode["odom_topic"]` 사용.

### 신규 REST 엔드포인트 3개

`create_app()` 안에 추가:

```
GET /telemetry/joints
  → {"names": ["FL_hip", "FL_thigh", "FL_calf", ...]}
  (데이터 미수신 시 {"names": []} 반환, 에러 아님)

GET /telemetry/joint?name=FL_hip&type=position
  → {"times": [...], "values": [...]}
  (type: "position" | "velocity", 기본값 "position")
  (존재하지 않는 name → 빈 배열 반환, 404 아님)

GET /telemetry/speed
  → {
      "cmd_linear":  {"times": [...], "values": [...]},
      "odom_linear": {"times": [...], "values": [...]},
      "cmd_angular": {"times": [...], "values": [...]},
      "odom_angular":{"times": [...], "values": [...]}
    }
```

**`now_sec` 계산 — 반드시 `time.monotonic()` 사용:**
```python
import time
now_sec = time.monotonic()
```
> ⚠️ TelemetryBridge 내부 타임스탬프는 전부 `time.monotonic()` 기준. `node.get_clock().now()` (ROS time)를 넘기면 X축이 수백만 초 단위로 틀어짐.

**스레드 안전 — 엔드포인트 핸들러에서 `_lock` 안에 wrapping:**
```python
@app.get("/telemetry/joint")
async def telemetry_joint(name: str = "", type: str = "position"):
    import time
    now_sec = time.monotonic()
    with node._lock:
        if type == "velocity":
            times, values = node.telemetry_bridge.get_joint_velocity_series(name, now_sec)
        else:
            times, values = node.telemetry_bridge.get_joint_position_series(name, now_sec)
    return {"times": times, "values": values}
```
> ⚠️ TelemetryBridge의 deque는 ROS spin 스레드에서 쓰이고 FastAPI asyncio 스레드에서 읽힘.  
> `_buffer_to_plot()` 내 리스트 컴프리헨션 순회 중 변경을 막기 위해 반드시 `_lock` 보호 필요.

**직렬화**: `_buffer_to_plot()` 반환값 `(x_times, y_values)` → `{"times": x_times, "values": y_values}`  
x_times 값은 음수 상대 시간 (예: `-19.5, -10.0, ..., 0.0` 초). Chart.js X축 레이블에 그대로 사용 가능.

---

## 2. 프론트엔드 변경 (`index.html`)

### 디자인 토큰 교체

| 항목 | 기존 | 변경 |
|------|------|------|
| 배경 | `#f2ede3` | `#f9fafb` (gray-50) |
| 패널 | `#fbf8f1` | `#ffffff` |
| 텍스트 | `#17342d` | `#111827` |
| 주요 accent | `#1f6a55` | `#2563eb` (blue-600) |
| 보조 accent | `#ad7442` | `#0ea5e9` (sky-500) |
| 위험 | `#b65042` | `#dc2626` (red-600) |
| 보더 | 암시적 | `#e5e7eb` (gray-200) |

폰트는 현재와 동일 유지 (Noto Sans KR, Pretendard).

### Charts 탭 추가 (4번째 탭)

탭 순서: Control | Runtime | Logs | **Charts**

**레이아웃:**

```
┌─────────────────────────────────────────────┐
│ Joint                                        │
│ Joint: [FL_hip ▼]  Type: [Position ▼]        │
│ ┌───────────────────────────────────────────┐│
│ │  Chart.js Line chart (position/velocity)  ││
│ │  X: 지난 20초 (relative)  Y: rad / rad/s  ││
│ └───────────────────────────────────────────┘│
├─────────────────────────────────────────────┤
│ Speed: cmd vs odom                           │
│ ┌───────────────────────────────────────────┐│
│ │  cmd_linear ──  odom_linear ──            ││
│ │  cmd_angular ─ ─  odom_angular ─ ─        ││
│ └───────────────────────────────────────────┘│
└─────────────────────────────────────────────┘
```

### Chart.js 통합

Chart.js 4.x `chart.umd.min.js` (~200KB) 를 `index.html` 하단 `<script>` 태그에 인라인 번들.
외부 CDN 없음, 단일 파일 유지.

소스 다운로드:
```
https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js
```
위 URL의 내용을 `</body>` 직전 `<script>` 태그에 직접 삽입.

### 폴링 로직

```javascript
let chartInterval = null;

function onTabChange(tab) {
    if (tab === "charts") {
        loadJointNames();  // GET /telemetry/joints → joint 드롭다운 채우기
        chartInterval = setInterval(refreshCharts, 500);
    } else {
        clearInterval(chartInterval);
        chartInterval = null;
    }
}

async function refreshCharts() {
    const jointName = jointSelector.value;
    const type = typeSelector.value;  // "position" | "velocity"
    const [jointData, speedData] = await Promise.all([
        fetch(`/telemetry/joint?name=${jointName}&type=${type}`).then(r => r.json()),
        fetch('/telemetry/speed').then(r => r.json()),
    ]);
    updateJointChart(jointData);
    updateSpeedChart(speedData);
}
```

---

## 구현 순서

1. `web_app.py` — TelemetryBridge 초기화 + 3개 엔드포인트 추가
2. `index.html` — 디자인 토큰 전면 교체 (색상/보더/그림자)
3. `index.html` — Charts 탭 DOM 추가 (joint selector, type selector, 두 canvas)
4. `index.html` — Chart.js 인라인 번들 삽입
5. `index.html` — 폴링 JS 로직 구현

---

## 검증 방법

```bash
colcon build --packages-select go2_gui_controller
source install/setup.bash
ros2 run go2_gui_controller web_controller --ros-args -p mode:=sim
# 브라우저: http://127.0.0.1:8080
```

확인 항목:
1. 전체 탭이 클린 화이트 디자인으로 렌더링
2. Charts 탭 → joint 드롭다운에 관절명 목록 로드
3. Charts 탭 활성 시 500ms 간격으로 그래프 갱신
4. 다른 탭 전환 시 폴링 중단 (Network 탭에서 확인)
5. `/telemetry/joints`, `/telemetry/joint`, `/telemetry/speed` 엔드포인트 정상 응답

---

## 추후 확장 항목

| 항목 | 방법 |
|------|------|
| 맵 시각화 | `/map` OccupancyGrid WebSocket → `<canvas>` 렌더링 |
| 클릭 목표 지점 | 맵 canvas 클릭 → 픽셀 → map 좌표 변환 → `POST /cmd/navigate_pose` |
| 카메라 피드 | `/camera/color/compressed` → MJPEG 스트림 또는 base64 WebSocket |
| 다크 모드 토글 | CSS 변수 기반 테마 전환 |
