---
tags: [project, implementation-plan, semantic-navigation, object-navigation]
status: in-progress
project: go2_intelligence_framework
type: implementation-plan
created: 2026-04-27
---

# 12. Semantic Object Navigation Plan

> Deep-interview + ralplan 결과를 반영한 실행 계획

## Progress Snapshot - 2026-04-28

### 완료

- Web GUI object navigation 경로 구현
  - `"소파로 가"`, `"노트북으로 가"`, `"벤치로 가"` 같은 일반 객체 명령 파싱
  - semantic object lookup
  - 객체 중심점 대신 approach pose 계산
  - Nav2 goal 전송
  - 객체 없음 실패 처리
- semantic map manifest / fail-closed 검증 구현
  - `map_id`, `source_fingerprint`, `frame_id` 검증
  - manifest 불일치 시 `semantic_manifest_mismatch`로 goal 전송 차단
- fixture semantic map 경로에서 Office semantic map 경로로 전환
  - `src/go2_gui_controller/config/semantic_objects.yaml`
  - 현재 Office map 기준 74개 객체 생성/연결
  - aliases: `소파`, `의자`, `모니터`, `식물`, `화분`, `책상`, `테이블`, `냉장고`, `노트북`, `벤치`, `꽃병` 등
- Web GUI 전용으로 범위 확정
  - Qt/Desktop GUI object navigation은 이번 단계에서 제외
- command parser 범위 정리
  - 새 환경/새 객체마다 파서를 다시 작성하지 않음
  - detector/semantic map이 label과 alias를 제공하고, parser는 일반 객체 명령 구조만 처리
- detector 방향 재결정 및 spike 완료
  - YOLO-World 직접 repo 설치는 의존성/운영 복잡도 때문에 1차 경로에서 제외
  - 1차 detector는 Ultralytics YOLO detect 모델로 진행
  - `models/yolo11n.pt` smoke/full RGB export inference 확인
  - segmentation은 필요 시 후속으로 전환 검토
- Isaac Sim Office USD 실험 시작
  - `assets/office.usd` 저장
  - `go2_sim.py` 기본 환경을 Office USD로 전환
  - Go2 spawn 위치를 환경변수로 조정 가능하게 변경
  - Isaac viewport와 충돌하지 않도록 수동 조작 키를 `I/K/J/L/U/O/P`로 변경
- Office USD 기준 SLAM/Nav 1차 확인
  - Office 환경에서 RTAB-Map DB 생성 확인
  - 생성한 Office DB로 Nav2 localization 실행 흐름 확인
  - 다음 단계 기준을 Office DB와 semantic object 좌표 정합으로 확정
- offline semantic map builder 구현
  - `scripts/build_semantic_map_from_yolo.py`
  - RTAB-Map export RGB/depth/calibration/camera pose 입력 처리
  - Ultralytics YOLO txt label 입력 처리
  - depth median 기반 bbox 중심 3D projection
  - camera pose를 이용한 `map` frame object coordinate 생성
  - label별 greedy XY merge
  - manifest 포함 `semantic_objects.yaml` 생성
- RTAB-Map Office DB export / YOLO inference / semantic map 생성 흐름 확인
  - `rtabmap-export --images_id --poses_camera ... maps/rtabmap_office.db`
  - `yolo predict model=models/yolo11n.pt ... save_txt=True save_conf=True`
  - generated Office semantic map을 Web GUI config로 연결
- semantic approach pose 방향 수정
  - 객체 좌표 자체가 아니라 객체를 관측했던 위치 방향의 접근점으로 이동
  - 예: `laptop_1 object=(7.8393,1.2171)`, `observer=(6.0504,2.6206)`, `goal=(7.0525,1.8344)`
- Web GUI runtime cleanup 추가
  - Runtime 탭 `Kill All`
  - `rtabmap_slam`, `nav2_`, `robot_state_publisher`, `rviz2` 일괄 정리

### 검증 완료

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_navigation.py
python3 -m unittest discover -s src/go2_gui_controller/test
python3 -m compileall src/go2_gui_controller/go2_gui_controller
python3 -m compileall src/go2_gui_controller/launch src/go2_gui_controller/setup.py
python3 -m compileall scripts/build_semantic_map_from_yolo.py
python3 -m compileall launch/go2_rtabmap.launch.py
colcon build --packages-select go2_gui_controller
python3 -m compileall scripts/go2_sim.py scripts/my_slam_env.py
```

### 현재 확인된 리스크 / 보류

- Office USD는 semantic detection 테스트용으로 유용하지만, 계단/복층 보행 테스트용으로 바로 신뢰하면 안 된다.
  - visual mesh와 collision mesh가 다를 수 있다.
  - 계단이 보여도 실제 물리 collision은 평평한 invisible collider일 수 있다.
- 다음 단계 전에 Office USD에서 사용할 테스트 구역을 정해야 한다.
  - 1차 semantic map 테스트는 리셉션/소파/의자/모니터/식물 같은 평면 구역 객체 중심으로 진행한다.
  - 계단/복층 이동은 별도 계획 또는 별도 USD collision 정리 후 진행한다.
- 현재 semantic navigation은 Nav2 planning 실패 시 다른 후보로 자동 fallback하지 않는다.
  - 예: `"벤치로 가"`는 confidence가 높은 `bench_1`을 선택했으나, goal `(-2.27, 9.95)`에 대해 Nav2 planner가 경로 생성 실패했다.
  - 다음 구현 단위는 같은 label의 다른 객체 후보 또는 다른 approach pose를 자동 재시도하는 fallback이다.
- approach pose가 실제 free space인지 아직 Nav2 costmap으로 사전검사하지 않는다.
  - 현재 validator는 frame/finite/distance/object radius margin만 확인한다.
  - costmap 기반 goal validation은 후속 단계로 남아 있다.
- semantic object 품질은 detector confidence와 관측 수에 따라 편차가 있다.
  - `observation_count=1` 객체는 실제 navigation 목표로 쓰기 전에 추가 검증이 필요하다.
- 문서의 Detector 결정 섹션은 초기 계획 기록으로 남긴다.
  - 실제 1차 구현은 YOLO-World가 아니라 Ultralytics YOLO detect 경로다.

---

## 목표

Web GUI에서 객체 기반 명령을 입력하면,
offline으로 생성된 semantic map에서 객체를 찾고,
해당 객체 주변의 접근 가능한 goal pose를 계산해 Nav2로 이동한다.

대표 데모:

```text
Web GUI 입력: "소파로 가"
  -> semantic object map에서 sofa/소파 검색
  -> sofa 중심점이 아니라 approach pose 계산
  -> Nav2 /navigate_to_pose goal 전송
```

객체가 없거나 semantic map이 현재 RTAB-Map 결과물과 맞지 않으면
goal을 보내지 않고 실패 처리한다.

---

## 핵심 방향

이번 단계는 **offline-first, online-ready**로 간다.

즉 1차 구현은 아래 흐름이다.

```text
1. Go2를 움직이며 RTAB-Map SLAM 수행
2. rtabmap.db / rosbag / exported RGB-D data 저장
3. offline semantic map builder 실행
4. semantic_objects.yaml 생성
5. Web GUI에서 "소파로 가" 테스트
```

SLAM 중 실시간으로 객체 탐지를 돌리는 online semantic mapper는
후속 단계로 남긴다.

이유:

- SLAM, detector, depth projection, TF sync, object merge, Nav2 실패 원인을 분리하기 쉽다.
- 같은 `rtabmap.db` 또는 rosbag으로 반복 테스트할 수 있다.
- RTAB-Map loop closure 이후 최적화된 map 기준으로 semantic object 좌표를 만들 수 있다.
- detector 선택과 semantic goal navigation을 분리해서 개발할 수 있다.

---

## 아키텍처

```text
[RTAB-Map DB / rosbag / RGB-D export]
          |
          v
Offline Semantic Map Builder
  - object detector adapter
  - depth projection
  - map-frame pose projection
  - duplicate merge
          |
          v
semantic_objects.yaml
  - manifest
  - object instances
          |
          v
Web GUI Text Command
  "소파로 가"
          |
          v
SemanticObjectRegistry
          |
          v
SemanticGoalResolver
  - object selection
  - relation / approach pose
  - safety validation
          |
          v
NavigatorBridge
  - Nav2 NavigateToPose
```

기존 RTAB-Map/Nav2 구조는 유지한다.
semantic map은 geometry map을 대체하지 않고,
`map` 좌표계 위에 객체 의미를 얹는 별도 layer로 둔다.

---

## Semantic Map 형식

1차 저장소는 YAML로 시작한다.
단, 단순 설정 파일처럼 취급하지 않는다.
semantic map은 특정 RTAB-Map 결과물에서 파생된 데이터이므로
반드시 manifest를 포함한다.

예시:

```yaml
manifest:
  map_id: "lab_map_20260427"
  source_rtabmap_db: "maps/rtabmap_ground_truth.db"
  source_fingerprint: "sha256:..."
  generated_at: "2026-04-27T03:00:00Z"
  builder_version: "semantic_builder_v1"
  frame_id: "map"
  detector:
    name: "yolo-world"
    model: "yolo-world-v1"

objects:
  sofa_1:
    label: "sofa"
    aliases: ["소파", "couch"]
    frame_id: "map"
    x: 2.0
    y: 3.0
    z: 0.0
    yaw_deg: 0.0
    radius_m: 0.4
    confidence: 0.86
    observation_count: 5
    updated_at: "2026-04-27T03:05:00Z"
    source: "offline_builder"
```

### Fail-Closed 정책

Web/Nav2 dispatch에서는 기본적으로 fail-closed다.

아래 경우에는 object goal을 보내지 않는다.

- manifest 없음
- `map_id` 불일치
- source fingerprint 불일치
- frame_id가 `map`이 아님
- stale semantic map으로 판단됨

warning-only 모드는 offline diagnostic 또는 테스트용 override에서만 허용한다.

---

## 1차 구현 범위

### 포함

- `SemanticObjectRegistry`
  - YAML load/reload
  - manifest validation
  - label/alias lookup
  - deterministic object selection

- `SemanticGoalResolver`
  - 객체 중심점 대신 approach pose 계산
  - `앞`, `뒤`, `왼쪽`, `오른쪽` 관계어 처리
  - default `"소파로 가"`는 near/front-style approach로 해석
  - goal pose safety validation

- Web GUI object command
  - `"소파로 가"`
  - `"소파 앞으로 가"`
  - `"소파 뒤로 가"`
  - `"소파 왼쪽으로 가"`
  - `"소파 오른쪽으로 가"`

- offline semantic map builder scaffold
  - fixture/exported detection input부터 시작 가능
  - detector adapter는 인터페이스로 분리
  - 1차 primary detector는 YOLO-World로 고정

### 제외

- SLAM 중 실시간 semantic map 업데이트
- 로봇 autonomous exploration
- full 3D scene graph
- 여러 객체 선택 UI
- GUI에서 사람이 객체 좌표를 찍어 등록하는 방식
- GroundingDINO + SAM2를 기본 필수 의존성으로 추가
- Desktop GUI object navigation

Desktop GUI는 1차 object navigation 범위에서 제외한다.
단, 기존 Desktop GUI 명령은 regression 없이 유지해야 한다.

---

## Object Selection 규칙

같은 label 객체가 여러 개 있으면 사용자에게 선택시키지 않는다.
1차에서는 deterministic rule을 사용한다.

우선순위:

1. confidence 높은 객체
2. confidence가 비슷하면 현재 robot pose에서 가까운 객체
3. 그래도 비슷하면 `updated_at`이 최신인 객체

선택된 객체 id는 log/status에 남긴다.

예:

```text
navigating_to_object: sofa_1
```

---

## Relation / Approach Pose 규칙

객체 중심점은 Nav2 goal로 부적절할 수 있다.
따라서 `"소파로 가"`도 object center가 아니라 approach pose로 변환한다.

기본 규칙:

- `front`, `back`, `left`, `right`는 object yaw 기준이다.
- object yaw가 없으면 observation approach metadata를 우선 사용한다.
- observation metadata도 없으면 현재 robot pose 기준 fallback을 사용하고 log에 남긴다.
- goal yaw는 기본적으로 object center를 바라보게 한다.

테스트용 고정 fixture:

```text
object center = (2.0, 3.0)
object yaw = 0 deg
radius = 0.4m
approach_distance = 1.0m

front  -> (3.0, 3.0)
back   -> (1.0, 3.0)
left   -> (2.0, 4.0)
right  -> (2.0, 2.0)
```

### 기본 safety validator

Nav2로 보내기 전 최소 검증을 수행한다.

- `frame_id == "map"`
- x/y/yaw 값이 finite
- approach distance 기본 범위: `0.6m <= d <= 1.5m`
- object radius가 있으면 `radius_m + 0.2m` safety margin 밖에 goal이 있어야 함
- 실패 시 `invalid_semantic_goal`로 goal 전송 차단

후속 단계에서는 이 validator를 occupancy map 또는 Nav2 costmap 검사와 연결한다.

---

## Detector 결정

초기 계획에서는 1차 offline semantic map builder의 primary detector를 **YOLO-World**로 정했다.
실제 구현 중 YOLO-World 직접 repo 설치가 `mmyolo`/OpenMMLab 의존성에서 부담이 커졌고,
이번 프로젝트의 1차 요구가 open-vocabulary prompt 실험보다
Office DB의 고정 객체 map 생성에 가까워졌기 때문에
현재 구현은 **Ultralytics YOLO detect (`models/yolo11n.pt`)** 경로로 전환했다.

현재 1차 구현 상태:

- `conda`의 `yolo` 환경에 Ultralytics 설치
- RTAB-Map RGB export에 대해 `yolo predict model=models/yolo11n.pt ... save_txt=True save_conf=True` 실행
- `scripts/build_semantic_map_from_yolo.py`가 Ultralytics txt label을 소비
- generated `semantic_objects.yaml`을 Web GUI config로 연결

아래 YOLO-World 내용은 초기 detector 검토 기록으로 남긴다.

선정 이유:

- open-vocabulary detection을 지원한다.
- bbox 기반 출력만으로도 depth projection과 semantic object 위치 추정이 가능하다.
- offline builder에서는 ROS2 node가 필수는 아니므로 Python image inference 중심으로 붙이기 쉽다.
- `"sofa"`, `"chair"`, `"desk"`, `"computer"` 같은 prompt vocabulary를 직접 실험하기 좋다.

1차 detector spike:

```text
1. YOLO-World 설치
2. 저장 이미지 또는 RGB frame fixture에서 inference
3. prompts: sofa, chair, desk, computer
4. bbox + confidence 출력 확인
5. builder adapter 출력 형식으로 변환
6. semantic_objects.yaml 생성까지 연결
```

Fallback:

- YOLO-World 설치/의존성 문제가 크면 YOLOE를 검토한다.
- mask/segmentation 기반 객체 중심/extent 추정이 필요해지면 YOLOE를 우선 검토한다.
- online ROS/Jetson 단계에서는 NanoOWL 또는 ROS2-NanoOWL을 별도 검토한다.

GroundingDINO + SAM2는 정확도 면에서 매력적이지만,
1차 기본 의존성으로는 무겁다.
필요 시 후속 고정밀 detector path로 검토한다.

---

## 수정 예상 파일

| 파일 | 작업 |
|------|------|
| `src/go2_gui_controller/go2_gui_controller/commands.py` | `NAVIGATE_TO_OBJECT` command type 및 ParsedCommand 필드 추가 |
| `src/go2_gui_controller/go2_gui_controller/text_command_parser.py` | object command 및 관계어 parsing 추가 |
| `src/go2_gui_controller/go2_gui_controller/semantic_object_registry.py` | 신규 semantic object registry |
| `src/go2_gui_controller/go2_gui_controller/semantic_goal_resolver.py` | 신규 approach pose resolver |
| `src/go2_gui_controller/go2_gui_controller/navigator_bridge.py` | map-frame semantic goal pose 전송 method 추가 |
| `src/go2_gui_controller/go2_gui_controller/web_app.py` | semantic registry 초기화 및 Web command 실행 연결 |
| `src/go2_gui_controller/launch/go2_web_controller.launch.py` | semantic map 관련 launch argument 추가 |
| `src/go2_gui_controller/config/semantic_objects.yaml` | fixture/default semantic object map |
| `src/go2_gui_controller/setup.py` | semantic object config 설치 항목 추가 |
| `docs/plan/08_semantic_navigation_direction.md` | 필요 시 구현 방향 링크 추가 |

추가 launch parameter:

```text
semantic_object_file
semantic_map_id
semantic_source_fingerprint
semantic_manifest_policy      # default: strict
semantic_manifest_override    # default: false
```

---

## 구현 단계

### Step 1. Registry / Resolver 단위 구현

- semantic object dataclass 정의
- manifest schema 정의
- YAML loader 작성
- label/alias lookup 작성
- object ranking 작성
- approach pose resolver 작성
- safety validator 작성

이 단계는 ROS runtime 없이 unit test 가능해야 한다.

### Step 2. Command Parser 확장

현재 parser는 `앞/뒤/왼쪽/오른쪽`을 relative movement로 먼저 처리한다.
object relation command가 relative movement로 잘못 해석되지 않도록
object command parsing을 generic relative movement branch보다 앞에 둔다.

기존 stop/cancel 우선순위는 유지한다.

### Step 3. Web Controller 연동

- `WebControllerNode`에 semantic registry 추가
- `/cmd/text` 실행 경로에서 object command 처리
- object not found, stale map, invalid goal 에러를 명확히 반환
- 기존 waypoint / relative movement / stop / cancel 동작 유지

### Step 4. Offline Builder Scaffold

처음부터 detector 전체를 붙이지 않아도 된다.

우선 fixture/exported detection 입력으로 semantic map을 생성한다.

예:

```text
detections.json + camera/map pose fixture
  -> semantic_objects.yaml
```

이후 RTAB-Map DB, rosbag, detector adapter를 단계적으로 붙인다.

### Step 5. YOLO-World Detector Spike

YOLO-World를 먼저 붙인다.

출력 contract:

```text
Detection(
  label="sofa",
  confidence=0.0~1.0,
  bbox=[x1, y1, x2, y2],
  frame_id/timestamp=...
)
```

이 contract를 depth projection 단계가 소비하도록 만든다.

YOLO-World 설치 또는 inference가 현재 환경에서 막히면,
그 사유를 기록하고 YOLOE fallback으로 전환한다.

---

## 테스트 계획

### Unit Test

- semantic registry manifest validation
- stale/mismatched map fail-closed
- label/alias lookup
- multiple object ranking
- relation별 approach pose 계산
- safety validator 실패 케이스
- parser regression

Parser regression:

```text
멈춰 -> stop
취소 -> cancel
앞으로 1미터 가 -> relative move
오른쪽으로 돌아 -> rotate
home -> waypoint
소파로 가 -> object navigation
소파 앞으로 가 -> object navigation + front
```

### Integration Test

- fake semantic registry + fake navigator로 Web command 실행
- object missing 시 object_not_found
- stale manifest 시 goal 전송 차단
- invalid approach pose 시 goal 전송 차단
- 기존 waypoint command는 여전히 `go_to_waypoint`

### Manual Demo

```text
1. RTAB-Map mapping 또는 기존 dataset 준비
2. offline semantic builder 실행
3. semantic_objects.yaml 생성 확인
4. Web controller를 semantic_object_file과 함께 실행
5. "소파로 가" 입력
6. Nav2 goal 전송 확인
7. 없는 객체 입력 시 실패 확인
8. "소파 앞/뒤/왼쪽/오른쪽으로 가" pose 차이 확인
```

---

## 성공 기준

- offline builder가 deterministic fixture에서 semantic object map을 생성한다.
- `semantic_objects.yaml` manifest가 현재 map identity와 일치해야 로드된다.
- Web GUI에서 `"소파로 가"`가 object navigation command로 해석된다.
- sofa 객체가 있으면 approach pose를 계산하고 Nav2 goal을 보낸다.
- sofa 객체가 없으면 `object_not_found`로 실패한다.
- stale semantic map이면 goal을 보내지 않는다.
- `"소파 앞/뒤/왼쪽/오른쪽"` 관계어가 deterministic pose로 변환된다.
- 기존 waypoint/relative/stop/cancel 명령은 regression 없이 동작한다.

---

## 후속 확장

1. **Online Semantic Mapper**
   - SLAM 중 RGB-D topic을 구독
   - 같은 registry writer API로 semantic map 업데이트

2. **Costmap 기반 Goal Validation**
   - approach pose가 실제 free space인지 Nav2 costmap으로 확인

3. **SQLite Backend**
   - observation history, confidence decay, merge history가 커지면 YAML에서 전환

4. **Object Disambiguation**
   - `"가까운 소파"`
   - `"거실 소파"`
   - `"두 번째 소파"`

5. **Scene Graph**
   - room/floor/object hierarchy
   - `"2층 복도에 있는 소파"` 같은 명령 처리

---

## 결정 요약

- 1차는 offline-first.
- semantic map은 RTAB-Map/Nav2를 대체하지 않는 별도 layer.
- YAML로 시작하되 manifest 필수.
- Web/Nav2 dispatch는 fail-closed.
- object center가 아니라 approach pose로 이동.
- 관계어는 최소 범위로 1차 포함.
- 1차 primary detector는 YOLO-World로 한다.
- YOLOE는 fallback, NanoOWL은 online/Jetson 후보로 남긴다.
- Desktop GUI object navigation은 후순위.
