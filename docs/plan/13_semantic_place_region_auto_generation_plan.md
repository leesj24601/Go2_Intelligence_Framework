# Semantic Place Region Auto Generation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** RTAB-Map/Nav2 2D map에서 방, 복도, 로비 같은 place/region 후보를 자동 생성하고, 기존 semantic object를 해당 place에 자동 소속시킨다.

**Architecture:** 기존 `semantic_objects.yaml`과 `SemanticObjectRegistry`를 유지하면서 `places` 레이어를 추가한다. 1차 자동 생성은 occupancy image 기반 `free-space segmentation -> watershed 분할 -> contour polygon 변환 -> room/corridor heuristic 분류`로 구현하고, 사람 개입은 후보 이름/alias 검수 수준으로 제한한다.

**Tech Stack:** Python 3, NumPy, Pillow, PyYAML, optional OpenCV/scikit-image fallback 검토, ROS2/Nav2 map YAML/PGM, existing `unittest` test style.

---

## 배경

현재 semantic map은 객체가 독립 좌표만 갖는다.

```text
objects:
  chair_1 -> x, y
  chair_2 -> x, y
  laptop_1 -> x, y
```

이 구조는 `"의자로 가"` 같은 명령에는 충분하지만, `"회의실 의자로 가"`, `"복도 끝으로 가"`, `"내가 있는 방의 책상으로 가"` 같은 명령에는 약하다.

다음 단계는 객체 위에 수동 그룹을 붙이는 것이 아니라, map에서 place 후보를 자동 생성하고 객체 좌표를 place polygon에 자동 배정하는 것이다.

---

## 핵심 원칙

- 수동 polygon 입력을 기본 경로로 두지 않는다.
- 자동 생성 결과는 deterministic 해야 한다.
- 첫 버전은 완벽한 room segmentation보다 재현 가능한 후보 생성과 디버깅 가능성을 우선한다.
- 사람이 하는 일은 `room_candidate_1`을 `"회의실"`로 이름 붙이거나 잘못된 후보를 비활성화하는 검수 수준으로 제한한다.
- Nav2 주행 가능 여부 판단은 계속 costmap/geometry layer가 담당한다.
- place layer는 goal grounding, object disambiguation, UI 표시, topological reasoning을 위한 semantic layer로 둔다.

---

## 목표 YAML 형식

기존 파일을 확장해 `manifest`, `places`, `objects`를 같은 문서에 둔다.

```yaml
manifest:
  map_id: rtabmap_office
  source_fingerprint: sha256:...
  frame_id: map
  source_rtabmap_db: maps/rtabmap_office.db
  source_nav2_map: maps/rtabmap_office.yaml
  builder_version: semantic_place_builder_v1

places:
  room_candidate_1:
    label: room
    aliases: []
    frame_id: map
    polygon:
      - [0.12, 1.88]
      - [4.08, 1.92]
      - [4.02, 5.76]
      - [0.16, 5.71]
    centroid_x: 2.08
    centroid_y: 3.82
    area_m2: 15.12
    confidence: 0.74
    source: auto_occupancy_watershed
    enabled: true

  corridor_candidate_1:
    label: corridor
    aliases: []
    frame_id: map
    polygon:
      - [4.1, 0.2]
      - [9.6, 0.3]
      - [9.5, 1.4]
      - [4.0, 1.3]
    centroid_x: 6.82
    centroid_y: 0.84
    area_m2: 6.11
    confidence: 0.68
    source: auto_occupancy_watershed
    enabled: true

objects:
  chair_1:
    label: chair
    aliases: [의자]
    frame_id: map
    x: 1.2
    y: 3.1
    place_id: room_candidate_1
```

`place_id`는 사람이 직접 쓰는 필드가 아니라, 객체 좌표와 place polygon으로 자동 계산한 캐시다.

---

## 자동 생성 파이프라인

```text
Nav2 map YAML/PGM 또는 RTAB-Map occupancy export
  -> map metadata 로드
  -> occupied/free/unknown threshold
  -> unknown 제거 및 free-space cleanup
  -> obstacle inflation으로 벽 주변 여유 제거
  -> distance transform 계산
  -> local maxima seed 추출
  -> watershed로 free space region 분할
  -> 작은 region 제거
  -> 길쭉한 region은 corridor, 넓은 region은 room 후보로 분류
  -> contour 추출
  -> Douglas-Peucker polygon simplification
  -> map 좌표계 polygon 변환
  -> semantic map의 places로 저장
  -> semantic objects의 place_id 자동 계산
```

1차 구현에서 기대하는 결과는 `room_candidate_N`, `corridor_candidate_N` 후보다. 실제 이름은 자동 생성하지 않는다. 이름 자동 생성은 map의 의미를 알 수 없기 때문에 `"회의실"`, `"로비"` 같은 alias 검수는 별도 metadata patch로 처리한다.

---

## 파일 구조

### 새 파일

- `scripts/build_semantic_places_from_occupancy.py`
  - Nav2 map YAML/PGM 또는 occupancy image를 입력받아 `places` 후보를 생성한다.
  - 기존 `semantic_objects.yaml`을 읽어 manifest를 보존하고 `places`, object `place_id`를 추가한 YAML을 출력한다.

- `src/go2_gui_controller/go2_gui_controller/semantic_place_geometry.py`
  - polygon 면적, centroid, point-in-polygon, bounding box, aspect ratio 계산을 담당한다.
  - ROS 의존성 없이 pure Python으로 유지한다.

- `src/go2_gui_controller/go2_gui_controller/semantic_place_registry.py`
  - YAML의 `places`를 로드하고 alias/place lookup을 담당한다.
  - `place_for_point(x, y)`로 객체 또는 로봇 위치가 어느 place에 속하는지 찾는다.

- `src/go2_gui_controller/test/test_semantic_place_geometry.py`
  - polygon geometry 단위 테스트.

- `src/go2_gui_controller/test/test_semantic_place_registry.py`
  - YAML 로드, alias lookup, point membership 테스트.

- `src/go2_gui_controller/test/test_semantic_place_builder.py`
  - 작은 synthetic occupancy image로 자동 후보 생성 테스트.

### 수정 파일

- `src/go2_gui_controller/go2_gui_controller/semantic_object_registry.py`
  - `SemanticObject.place_id: Optional[str]` 필드 추가.
  - YAML object entry의 `place_id`를 로드한다.
  - `best_match()`에 optional `place_id` 필터를 추가한다.

- `src/go2_gui_controller/go2_gui_controller/commands.py`
  - `ParsedCommand.place_label: Optional[str]` 추가.

- `src/go2_gui_controller/go2_gui_controller/text_command_parser.py`
  - `"회의실 의자로 가"`, `"복도에 있는 벤치로 가"` 형태에서 place label과 object label을 분리한다.

- `src/go2_gui_controller/go2_gui_controller/web_app.py`
  - object navigation 시 place label이 있으면 `SemanticPlaceRegistry`로 place를 찾고, `SemanticObjectRegistry.best_match(..., place_id=...)`로 후보를 제한한다.

- `src/go2_gui_controller/config/semantic_objects.yaml`
  - 자동 builder 결과물로 `places`와 object `place_id`가 추가될 수 있다.

---

## Task 1: Polygon Geometry 유틸리티 추가

**Files:**

- Create: `src/go2_gui_controller/go2_gui_controller/semantic_place_geometry.py`
- Create: `src/go2_gui_controller/test/test_semantic_place_geometry.py`

- [ ] **Step 1: 실패 테스트 작성**

```python
from __future__ import annotations

import unittest

from go2_gui_controller.semantic_place_geometry import (
    polygon_area,
    polygon_centroid,
    point_in_polygon,
)


class SemanticPlaceGeometryTests(unittest.TestCase):
    def test_point_membership_area_and_centroid(self):
        polygon = ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0))

        self.assertTrue(point_in_polygon(2.0, 1.5, polygon))
        self.assertFalse(point_in_polygon(5.0, 1.5, polygon))
        self.assertAlmostEqual(polygon_area(polygon), 12.0)
        self.assertEqual(polygon_centroid(polygon), (2.0, 1.5))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_geometry.py
```

Expected: `ModuleNotFoundError` 또는 import 실패.

- [ ] **Step 3: 최소 구현**

```python
from __future__ import annotations

from typing import Iterable

Point = tuple[float, float]


def point_in_polygon(x: float, y: float, polygon: Iterable[Point]) -> bool:
    points = list(polygon)
    inside = False
    j = len(points) - 1
    for i, (xi, yi) in enumerate(points):
        xj, yj = points[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def polygon_area(polygon: Iterable[Point]) -> float:
    points = list(polygon)
    total = 0.0
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        total += x1 * y2 - x2 * y1
    return abs(total) * 0.5


def polygon_centroid(polygon: Iterable[Point]) -> Point:
    points = list(polygon)
    signed_area_twice = 0.0
    cx = 0.0
    cy = 0.0
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        cross = x1 * y2 - x2 * y1
        signed_area_twice += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    if abs(signed_area_twice) < 1e-12:
        return (sum(x for x, _ in points) / len(points), sum(y for _, y in points) / len(points))
    return (cx / (3.0 * signed_area_twice), cy / (3.0 * signed_area_twice))
```

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_geometry.py
```

Expected: `OK`.

---

## Task 2: Place Registry 추가

**Files:**

- Create: `src/go2_gui_controller/go2_gui_controller/semantic_place_registry.py`
- Create: `src/go2_gui_controller/test/test_semantic_place_registry.py`

- [ ] **Step 1: 실패 테스트 작성**

```python
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from go2_gui_controller.semantic_place_registry import SemanticPlaceRegistry


FIXTURE_YAML = """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
places:
  room_candidate_1:
    label: room
    aliases: [회의실, 방1]
    frame_id: map
    polygon:
      - [0.0, 0.0]
      - [4.0, 0.0]
      - [4.0, 3.0]
      - [0.0, 3.0]
    enabled: true
  corridor_candidate_1:
    label: corridor
    aliases: [복도]
    frame_id: map
    polygon:
      - [4.0, 0.0]
      - [8.0, 0.0]
      - [8.0, 1.0]
      - [4.0, 1.0]
    enabled: true
"""


def _write_yaml(text: str) -> Path:
    temp_dir = tempfile.mkdtemp()
    path = Path(temp_dir) / "semantic_objects.yaml"
    path.write_text(text, encoding="utf-8")
    return path


class SemanticPlaceRegistryTests(unittest.TestCase):
    def test_lookup_and_point_membership(self):
        registry = SemanticPlaceRegistry(_write_yaml(FIXTURE_YAML))

        self.assertEqual(registry.best_match("회의실").place_id, "room_candidate_1")
        self.assertEqual(registry.best_match("복도").place_id, "corridor_candidate_1")
        self.assertEqual(registry.place_for_point(2.0, 1.5).place_id, "room_candidate_1")
        self.assertEqual(registry.place_for_point(6.0, 0.5).place_id, "corridor_candidate_1")
        self.assertIsNone(registry.place_for_point(9.0, 9.0))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_registry.py
```

Expected: `ModuleNotFoundError` 또는 import 실패.

- [ ] **Step 3: 구현**

`SemanticPlace` dataclass 필드는 아래로 고정한다.

```python
place_id: str
label: str
aliases: tuple[str, ...]
frame_id: str
polygon: tuple[tuple[float, float], ...]
centroid_x: float
centroid_y: float
area_m2: float
confidence: float
source: str
enabled: bool
```

`SemanticPlaceRegistry`는 기존 `SemanticObjectRegistry` 패턴을 따른다.

- YAML이 없으면 빈 registry.
- `places`가 없으면 빈 registry.
- `enabled: false` place는 lookup과 membership에서 제외.
- alias lookup은 소문자 normalized key 사용.
- `place_for_point(x, y)`는 point가 들어간 enabled place 중 면적이 가장 작은 place를 반환한다. 겹치는 polygon이 있을 때 작은 방이 큰 zone보다 우선되어야 한다.

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_registry.py
```

Expected: `OK`.

---

## Task 3: Semantic Object에 place_id 추가

**Files:**

- Modify: `src/go2_gui_controller/go2_gui_controller/semantic_object_registry.py`
- Modify: `src/go2_gui_controller/test/test_semantic_navigation.py`

- [ ] **Step 1: 실패 테스트 추가**

`FIXTURE_YAML`의 `sofa_1`에 `place_id: room_candidate_1`, `sofa_old`에 `place_id: room_candidate_2`를 추가하고 아래 테스트를 넣는다.

```python
def test_best_match_can_filter_by_place(self):
    registry = SemanticObjectRegistry(_write_registry(FIXTURE_YAML))

    self.assertEqual(registry.best_match("소파", place_id="room_candidate_1").object_id, "sofa_1")
    self.assertEqual(registry.best_match("소파", place_id="room_candidate_2").object_id, "sofa_old")
    self.assertIsNone(registry.best_match("소파", place_id="missing_room"))
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_navigation.py
```

Expected: `TypeError` because `best_match()` does not accept `place_id`.

- [ ] **Step 3: 구현**

`SemanticObject`에 필드 추가:

```python
place_id: Optional[str] = None
```

`_parse_object()`에서 추가:

```python
place_id=str(raw.get("place_id", "")).strip() or None,
```

`best_match()` signature 변경:

```python
def best_match(
    self,
    label_or_alias: str,
    *,
    current_pose: tuple[float, float] | None = None,
    place_id: str | None = None,
) -> SemanticObject | None:
```

후보 필터:

```python
if place_id:
    candidates = [obj for obj in candidates if obj.place_id == place_id]
```

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_navigation.py
```

Expected: `OK`.

---

## Task 4: Occupancy 기반 Place Builder 추가

**Files:**

- Create: `scripts/build_semantic_places_from_occupancy.py`
- Create: `src/go2_gui_controller/test/test_semantic_place_builder.py`

- [ ] **Step 1: synthetic map 실패 테스트 작성**

테스트는 외부 ROS 실행 없이 작은 occupancy image를 만든다.

```python
from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build_semantic_places_from_occupancy.py"


class SemanticPlaceBuilderTests(unittest.TestCase):
    def test_builds_places_and_assigns_objects(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            map_image = root / "map.pgm"
            map_yaml = root / "map.yaml"
            semantic_input = root / "semantic_objects.yaml"
            output = root / "semantic_with_places.yaml"

            # 0=occupied, 254=free, 205=unknown in common Nav2 map convention.
            pixels = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 254, 254, 254, 0, 254, 254, 0],
                [0, 254, 254, 254, 0, 254, 254, 0],
                [0, 254, 254, 254, 254, 254, 254, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
            ]
            image = Image.new("L", (8, 5))
            image.putdata([value for row in pixels for value in row])
            image.save(map_image)
            map_yaml.write_text(
                "image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
                encoding="utf-8",
            )
            semantic_input.write_text(
                """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
objects:
  chair_1:
    label: chair
    aliases: [의자]
    frame_id: map
    x: 2.0
    y: 2.0
""",
                encoding="utf-8",
            )

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--map-yaml",
                    str(map_yaml),
                    "--semantic-input",
                    str(semantic_input),
                    "--output",
                    str(output),
                    "--min-region-area-m2",
                    "2.0",
                ],
                check=True,
                cwd=ROOT,
            )

            data = yaml.safe_load(output.read_text(encoding="utf-8"))
            self.assertIn("places", data)
            self.assertGreaterEqual(len(data["places"]), 1)
            self.assertEqual(data["objects"]["chair_1"]["place_id"], next(iter(data["places"].keys())))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_builder.py
```

Expected: script file missing.

- [ ] **Step 3: Builder 구현**

CLI 인자:

```text
--map-yaml PATH
--semantic-input PATH
--output PATH
--min-region-area-m2 FLOAT default=1.0
--inflate-cells INT default=1
--simplify-tolerance-m FLOAT default=0.25
--corridor-aspect-ratio FLOAT default=3.0
```

1차 구현은 dependency 부담을 줄이기 위해 pure Python/NumPy/Pillow로 시작한다.

- map YAML에서 `image`, `resolution`, `origin`, `occupied_thresh`, `free_thresh`, `negate`를 읽는다.
- image를 grayscale로 로드한다.
- free mask를 만든다.
- occupied 주변 `inflate-cells`만큼 free를 제거한다.
- connected components를 만든다.
- 각 component contour를 bounding rectangle 기반 polygon으로 시작한다.
- component의 width/height 비율이 `corridor-aspect-ratio` 이상이면 `corridor`, 아니면 `room`으로 분류한다.
- output place id는 `room_candidate_1`, `corridor_candidate_1`처럼 deterministic하게 생성한다.
- object `x`, `y`가 place polygon 안에 있으면 `place_id`를 기록한다.

이 Task는 watershed까지 가지 않는다. 먼저 자동 place layer의 end-to-end 흐름을 만든다.

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_builder.py
```

Expected: `OK`.

---

## Task 5: Watershed 기반 분할 고도화

**Files:**

- Modify: `scripts/build_semantic_places_from_occupancy.py`
- Modify: `src/go2_gui_controller/test/test_semantic_place_builder.py`

- [ ] **Step 1: 실패 테스트 추가**

하나의 connected component 안에 좁은 연결부로 이어진 두 free-space blob을 만든다. 테스트는 builder가 한 connected component를 최소 2개 place 후보로 나누는지 확인한다.

```python
def test_splits_connected_rooms_through_narrow_passage(self):
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        map_image = root / "map.pgm"
        map_yaml = root / "map.yaml"
        semantic_input = root / "semantic_objects.yaml"
        output = root / "semantic_with_places.yaml"

        pixels = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 254, 254, 254, 0, 254, 254, 254, 0],
            [0, 254, 254, 254, 0, 254, 254, 254, 0],
            [0, 254, 254, 254, 254, 254, 254, 254, 0],
            [0, 254, 254, 254, 0, 254, 254, 254, 0],
            [0, 254, 254, 254, 0, 254, 254, 254, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0],
        ]
        image = Image.new("L", (9, 7))
        image.putdata([value for row in pixels for value in row])
        image.save(map_image)
        map_yaml.write_text(
            "image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        semantic_input.write_text(
            """
manifest:
  map_id: fixture_map
  source_fingerprint: fixture
  frame_id: map
objects: {}
""",
            encoding="utf-8",
        )

        subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--map-yaml",
                str(map_yaml),
                "--semantic-input",
                str(semantic_input),
                "--output",
                str(output),
                "--min-region-area-m2",
                "2.0",
            ],
            check=True,
            cwd=ROOT,
        )

        data = yaml.safe_load(output.read_text(encoding="utf-8"))
        enabled_places = [place for place in data["places"].values() if place.get("enabled", True)]
        self.assertGreaterEqual(len(enabled_places), 2)
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_builder.py
```

Expected: connected component baseline이 place 1개만 생성해서 실패.

- [ ] **Step 3: 거리 변환과 seed 기반 분할 구현**

구현 순서:

1. free cell마다 가장 가까운 occupied cell까지 Manhattan distance를 계산한다.
2. distance가 지역 최대인 cell을 seed로 삼는다.
3. seed 간 거리가 너무 가까우면 confidence가 높은 seed만 남긴다.
4. 각 free cell을 가장 가까운 seed에 배정한다.
5. seed별 region을 component처럼 polygon으로 변환한다.

OpenCV 또는 scikit-image를 바로 필수 의존성으로 추가하지 않는다. 이 프로젝트의 `requirements.txt` 변경은 별도 검토가 필요하므로, 1차는 NumPy 기반 deterministic algorithm으로 둔다.

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_builder.py
```

Expected: `OK`.

---

## Task 6: Place-aware 명령 파싱

**Files:**

- Modify: `src/go2_gui_controller/go2_gui_controller/commands.py`
- Modify: `src/go2_gui_controller/go2_gui_controller/text_command_parser.py`
- Modify: `src/go2_gui_controller/test/test_semantic_navigation.py`

- [ ] **Step 1: 실패 테스트 추가**

```python
def test_place_scoped_object_command_parses(self):
    parser = TextCommandParser(_FakeWaypointRegistry())

    command = parser.parse("회의실 의자로 가")

    self.assertEqual(command.command_type, CommandType.NAVIGATE_TO_OBJECT)
    self.assertEqual(command.place_label, "회의실")
    self.assertEqual(command.object_label, "의자")
    self.assertEqual(command.object_relation, "near")
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_navigation.py
```

Expected: `ParsedCommand`에 `place_label` 없음.

- [ ] **Step 3: 구현**

`ParsedCommand`에 필드 추가:

```python
place_label: Optional[str] = None
```

`TextCommandParser._extract_object_goal()`은 반환값을 `tuple[str | None, str, str]`로 확장한다.

파싱 규칙:

- `"회의실 의자로 가"` -> place=`회의실`, object=`의자`
- `"회의실에 있는 의자로 가"` -> place=`회의실`, object=`의자`
- `"복도 벤치로 가"` -> place=`복도`, object=`벤치`
- `"소파로 가"` -> place=None, object=`소파`

1차 규칙은 2-token 이상인 한국어 명령에서 첫 token을 place candidate로, 나머지를 object candidate로 본다. 실제 place 존재 여부는 parser가 아니라 registry 단계에서 검증한다.

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_navigation.py
```

Expected: `OK`.

---

## Task 7: Web command 실행에 Place Registry 연결

**Files:**

- Modify: `src/go2_gui_controller/go2_gui_controller/web_app.py`
- Create: `src/go2_gui_controller/test/test_web_semantic_place_execution.py`

- [ ] **Step 1: 실패 테스트 작성**

`web_app.py` import가 ROS2 모듈에 의존하므로 테스트 파일에서 필요한 모듈을 stub 처리한다. 핵심 검증은 `execute_parsed_command()`가 place lookup 결과를 object lookup filter로 넘기는지다.

```python
from __future__ import annotations

import sys
import types
import unittest


def _install_ros_stubs() -> None:
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = type("Twist", (), {})
    geometry_msgs_msg.PoseStamped = type("PoseStamped", (), {})
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: False
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy.node = rclpy_node
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)

    action_msgs = types.ModuleType("action_msgs")
    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = type("GoalStatus", (), {})
    action_msgs.msg = action_msgs_msg
    sys.modules.setdefault("action_msgs", action_msgs)
    sys.modules.setdefault("action_msgs.msg", action_msgs_msg)


_install_ros_stubs()

from go2_gui_controller.commands import CommandType, ParsedCommand
from go2_gui_controller.web_app import execute_parsed_command


class _PlaceRegistry:
    def best_match(self, label):
        self.requested_label = label
        return types.SimpleNamespace(place_id="room_candidate_1")


class _ObjectRegistry:
    def validate_manifest(self, **_kwargs):
        return None

    def best_match(self, label, *, current_pose=None, place_id=None):
        self.requested = (label, current_pose, place_id)
        return types.SimpleNamespace(
            object_id="chair_1",
            frame_id="map",
            x=2.0,
            y=3.0,
            z=0.0,
            radius_m=0.4,
            yaw_deg=None,
            approach_yaw_deg=None,
            observer_x=1.0,
            observer_y=3.0,
        )


class _Resolver:
    def resolve(self, semantic_object, relation, *, current_pose=None):
        self.requested = (semantic_object.object_id, relation, current_pose)
        return types.SimpleNamespace(
            object_id=semantic_object.object_id,
            relation=relation,
            frame_id="map",
            x=1.0,
            y=3.0,
            yaw_rad=0.0,
        )


class _Navigator:
    def go_to_semantic_goal(self, goal):
        self.goal = goal


class WebSemanticPlaceExecutionTests(unittest.TestCase):
    def test_place_label_filters_object_lookup(self):
        object_registry = _ObjectRegistry()
        place_registry = _PlaceRegistry()
        resolver = _Resolver()
        navigator = _Navigator()
        logs = []
        node = types.SimpleNamespace(
            semantic_map_id="fixture_map",
            semantic_source_fingerprint="fixture",
            semantic_manifest_policy="strict",
            semantic_manifest_override=False,
            semantic_registry=object_registry,
            semantic_place_registry=place_registry,
            semantic_goal_resolver=resolver,
            navigator_bridge=navigator,
            state_bridge=types.SimpleNamespace(state=types.SimpleNamespace(frame_id="map", x=0.0, y=0.0)),
            _append_log=logs.append,
        )
        command = ParsedCommand(
            CommandType.NAVIGATE_TO_OBJECT,
            object_label="의자",
            object_relation="near",
            place_label="회의실",
            source_text="회의실 의자로 가",
        )

        result = execute_parsed_command(node, command, source_prefix="text")

        self.assertEqual(result, "navigating to object chair_1")
        self.assertEqual(place_registry.requested_label, "회의실")
        self.assertEqual(object_registry.requested, ("의자", (0.0, 0.0), "room_candidate_1"))
        self.assertEqual(resolver.requested, ("chair_1", "near", (0.0, 0.0)))
        self.assertEqual(navigator.goal.object_id, "chair_1")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run:

```bash
python3 -m unittest discover -s src/go2_gui_controller/test
```

Expected: place-aware execution path 미구현으로 실패.

- [ ] **Step 3: 구현**

`WebControllerNode.__init__`에서 기존 semantic file을 `SemanticPlaceRegistry`에도 전달한다.

```python
self.semantic_place_registry = SemanticPlaceRegistry(semantic_object_file)
```

`execute_parsed_command()` object navigation 분기에서:

```python
place_id = None
if command.place_label:
    place = node.semantic_place_registry.best_match(command.place_label)
    if place is None:
        raise ValueError(f"place_not_found:{command.place_label}")
    place_id = place.place_id

semantic_object = node.semantic_registry.best_match(
    command.object_label,
    current_pose=current_pose,
    place_id=place_id,
)
```

로그는 place filter를 포함한다.

```text
[Control] text navigate_to_object: chair_1 place=room_candidate_1 relation=near x=...
```

- [ ] **Step 4: 통과 확인**

Run:

```bash
python3 -m unittest discover -s src/go2_gui_controller/test
python3 -m compileall src/go2_gui_controller/go2_gui_controller
```

Expected: `OK`, compile success.

---

## Task 8: Office map 자동 생성 smoke 실행 절차 문서화

**Files:**

- Modify: `docs/plan/12_semantic_object_navigation_plan.md`
- Modify: `docs/project_overview.md`

- [ ] **Step 1: 실행 명령 추가**

Office map 기준 smoke command를 문서에 추가한다.

```bash
python3 scripts/build_semantic_places_from_occupancy.py \
  --map-yaml maps/rtabmap_office.yaml \
  --semantic-input src/go2_gui_controller/config/semantic_objects.yaml \
  --output outputs/semantic_objects_with_places.yaml \
  --min-region-area-m2 1.0 \
  --inflate-cells 1
```

검증 명령:

```bash
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_geometry.py
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_registry.py
python3 -m unittest src/go2_gui_controller/test/test_semantic_place_builder.py
python3 -m unittest discover -s src/go2_gui_controller/test
python3 -m compileall scripts/build_semantic_places_from_occupancy.py
python3 -m compileall src/go2_gui_controller/go2_gui_controller
```

- [ ] **Step 2: 문서 검토**

문서에서 수동 polygon 등록을 필수 경로로 설명하는 표현을 제거한다. 수동 보정은 optional metadata patch로만 언급한다.

---

## 후속 개선 범위

이 계획의 1차 완료 조건에는 포함하지 않는다.

- RViz 클릭 기반 polygon editor.
- Web GUI polygon drawing tool.
- VLM으로 `"회의실"`, `"주방"` 같은 place 이름 자동 부여.
- 3D point cloud 기반 floor/room segmentation.
- door detector와 topological graph 자동 생성.
- Nav2 costmap service를 통한 place entrance pose 자동 검증.

이 항목들은 자동 생성 파이프라인이 동작한 뒤, 품질 병목이 확인될 때 별도 계획으로 분리한다.

---

## 완료 기준

- `scripts/build_semantic_places_from_occupancy.py`가 synthetic map에서 deterministic place 후보를 생성한다.
- 생성된 YAML에 `places`가 있고 기존 `objects`에 `place_id`가 자동 부여된다.
- `SemanticPlaceRegistry`가 place alias lookup과 point membership을 지원한다.
- `"회의실 의자로 가"` 같은 place-scoped object command가 parsing된다.
- Web command 실행 시 place filter가 object 후보 선택에 반영된다.
- 기존 `"소파로 가"` object navigation은 place 정보 없이 계속 동작한다.
- 아래 명령이 모두 성공한다.

```bash
python3 -m unittest discover -s src/go2_gui_controller/test
python3 -m compileall scripts/build_semantic_places_from_occupancy.py
python3 -m compileall src/go2_gui_controller/go2_gui_controller
```

---

## 리스크와 대응

- 자동 room/corridor 분할은 map 품질과 threshold에 민감하다.
  - 대응: synthetic map 테스트와 Office map smoke 결과를 분리해서 본다.

- connected component만으로는 열린 문으로 연결된 방이 하나로 합쳐질 수 있다.
  - 대응: Task 5에서 distance seed 기반 분할을 추가한다.

- room/corridor 자동 분류는 aspect ratio heuristic만으로 틀릴 수 있다.
  - 대응: `label`은 후보값으로 두고 `aliases` 검수 단계에서 사람이 수정할 수 있게 한다.

- polygon이 실제 free space 전체와 정확히 일치하지 않을 수 있다.
  - 대응: place polygon은 semantic 후보 영역으로만 사용하고, 실제 navigation safety는 Nav2 costmap 검증 단계에서 처리한다.

- parser가 `"회의실 의자"`에서 어느 단어가 place인지 완벽히 알 수 없다.
  - 대응: parser는 candidate만 분리하고, 실제 place 존재 여부는 `SemanticPlaceRegistry`가 검증한다.
