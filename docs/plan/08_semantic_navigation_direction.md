---
tags: [project, research, semantic-navigation]
status: research
project: go2_intelligence_framework
type: research-notes
created: 2026-02-24
---

# 08. Semantic Navigation Direction (2026)

## Goal

현재 프로젝트는 RTAB-Map + Nav2 + GUI Controller를 통해

- 기하학적 지도 생성
- Goal Pose 기반 자율주행
- waypoint / 상대이동 기반 텍스트 명령

까지는 구현되어 있다.

다음 단계에서 검토할 주제는,

- `"소파로 가"`
- `"소파 앞으로 가"`
- `"책상 오른쪽으로 가"`

처럼 **객체 의미를 포함한 자연어 명령**을 어떻게 처리할 것인가이다.

이 문서는 그에 대한 개념 정리와,
**2026년 기준으로 가장 현실적이고 주목받는 구현 방향**을 정리한 작업 문서다.

---

## 현재 구조의 한계

현재 GUI 텍스트 명령은 본질적으로 아래 두 종류만 처리한다.

1. 상대 이동
   - 예: `앞으로 1미터 가`
   - 현재 pose 기준으로 target pose 계산

2. waypoint 이름 매칭
   - 예: `go to home`
   - waypoint registry에 저장된 좌표로 goal 전송

즉 현재 구조는
**문장을 pose 또는 waypoint 이름으로 변환하는 계층**이며,
`map` 안의 객체를 이해하는 구조는 아니다.

따라서 현재 구조만으로는 아래 명령을 처리할 수 없다.

- `소파로 가`
- `문 앞까지 가`
- `책상 오른쪽으로 가`

이유는 단순하다.

- RTAB-Map / Nav2의 기본 지도는 `geometry map`
- 즉 벽, 장애물, 빈 공간, 로봇 위치는 다루지만
- `이 물체가 소파다`라는 **semantic label**은 포함하지 않는다

사람 눈에는 point cloud나 occupancy map이 소파처럼 보여도,
로봇 입장에서는 그냥 점/면/장애물일 뿐이다.

---

## 핵심 개념 정리

### 1. Geometry Map

SLAM이 만드는 기본 지도다.

- 벽
- 기둥
- 장애물
- 주행 가능 공간
- 로봇 위치

를 다루며,
Nav2가 경로 계획을 할 때 직접 사용하는 지도 계층이다.

### 2. Semantic Layer

기본 지도 위에 의미 정보를 덧붙인 계층이다.

예:

- `여기에 sofa가 있다`
- `이 객체는 desk다`
- `이 문 앞 접근 지점은 여기다`

중요한 점은,
semantic layer는 보통 **SLAM을 다시 하는 것**이 아니라
기존 `map` 좌표계 위에 객체 의미를 주석처럼 붙이는 것이다.

즉:

- geometry map = 공간 구조
- semantic layer = 객체 의미

### 3. Semantic SLAM

더 강한 개념이다.

geometry와 semantic을 더 깊게 결합해,
맵 생성 과정 자체에 객체 의미를 함께 넣는다.

하지만 현재 실무/개발에서 가장 흔한 방식은
이보다 가벼운 구조다.

- geometry SLAM은 그대로 유지
- semantic 정보는 별도 memory/map/scene graph로 얹음

즉 현재 주류는 보통
**"semantic SLAM 하나로 끝내기"보다 "geometry map + semantic memory"** 쪽이다.

---

## 2026년 기준 주류 구현 방식

현재 가장 실용적이고 많이 선택되는 방향은 아래와 같다.

### 아키텍처

```text
[RGB / RGB-D / Camera]
   |
   v
Open-vocabulary perception
   - YOLO-World
   - Grounding DINO
   - NanoOWL / OWL-ViT
   - SAM2 (optional, mask refinement)
   |
   v
Projection to map frame
   - depth
   - TF
   - robot pose
   |
   v
Semantic memory / semantic map / 3D scene graph
   - sofa_1
   - desk_2
   - door_1
   |
   v
Goal grounding
   - "소파로 가" -> sofa instance 검색
   - "소파 앞으로 가" -> approach pose 계산
   |
   v
Nav2
   - reachable goal pose 생성
   - path planning
   - obstacle avoidance
```

핵심은 이것이다.

- 객체를 찾는 일은 perception/VLM이 담당
- 의미를 저장하는 일은 semantic memory가 담당
- 실제 이동은 Nav2가 담당

즉 `인지`와 `이동`을 분리하는 구조가 현재 가장 설득력 있다.

---

## 왜 semantic layer가 사실상 필요한가

질문의 핵심은 이거다.

> 이미 SLAM으로 3D map을 만들었는데,
> 왜 또 semantic 정보를 따로 쌓아야 하나?

답은:
**현재의 geometry map만으로는 객체 의미를 안정적으로 알 수 없기 때문**이다.

SLAM 지도는 보통 아래는 잘 한다.

- 구조 복원
- 위치 추정
- free space 계산
- 장애물 회피

하지만 아래는 잘 못 한다.

- 저게 소파인지
- 책상인지
- TV인지
- "앞", "오른쪽"이 어디인지

그래서 현재는 거의 항상 아래 중 하나가 필요하다.

1. 명령 시점에 카메라로 객체를 다시 찾는다
2. 평소에 객체를 관측하며 semantic memory를 쌓는다
3. 둘을 섞는다

실무적으로는 3번이 가장 일반적이다.

- 평소에 약하게 기억
- 명령 시점에 재검증

---

## "3D map만 보고 인간처럼 이해"는 가능한가

최종적으로 더 강력하고 멋진 방향은 맞다.

즉,

- raw 3D map
- point cloud
- multi-view reconstruction

만 보고
`저건 소파다`를 인간처럼 이해하는 것이 궁극적으로 더 강한 지능이다.

하지만 2026년 현재 기준으로는
**부분적으로는 가능하지만, 일반적이고 안정적인 표준 구현은 아직 아니다.**

왜 아직 주류가 아니냐면:

1. 일반화가 어렵다
   - 환경이 바뀌면 성능이 흔들림

2. 실시간성이 부담된다
   - 거대한 3D reasoning / VLM은 온보드 로봇에서 무거움

3. 디버깅이 어렵다
   - 실패 원인이 perception인지 planning인지 분리하기 어려움

4. 접근 pose 계산이 별도 문제다
   - 소파를 알아도 `어디까지 가야 하는지`는 또 다른 문제

따라서 현재 주류는
`맵 자체를 end-to-end로 완전히 이해하는 단일 거대 모델`
보다는,
`geometry + perception + semantic memory + classical navigation`
조합이다.

---

## 모델 크기 관점에서의 현실성

자연스럽게 드는 질문은 이것이다.

> 그러면 semantic layer용 모델은 가볍고,
> 3D map 자체를 인간처럼 이해하는 모델은 더 무거운가?

대체로 맞다.

### 비교

1. Semantic layer 구축
   - open-vocabulary detector
   - optional segmentation
   - depth + TF projection
   - object memory 저장

이 방식은 비교적 가볍고,
현재 ROS2 / Nav2 구조에 붙이기 쉽다.

2. 3D map end-to-end 이해
   - 거대한 VLM / 3D reasoning
   - multi-view understanding
   - relation reasoning
   - direct goal grounding

이 방식은 더 지능적으로 보이지만,
계산량과 구현 복잡도가 훨씬 높다.

즉 현재 기준으로는
**semantic layer 구축 방식이 더 현실적이고 엔지니어링 친화적**이다.

---

## 이 프로젝트에서 추천하는 방향

현재 저장소 기준으로 가장 맞는 방향은 아래다.

### 유지할 것

- RTAB-Map
- Nav2
- 기존 GUI Controller
- `/goal_pose` 기반 goal 전송 구조

### 추가할 것

1. 객체 인식 계층
   - RGB / RGB-D 입력에서 `sofa`, `desk`, `door` 탐지

2. map frame 투영
   - detection 결과를 depth/TF로 `map` 좌표계에 올림

3. semantic memory
   - 객체 라벨, 위치, confidence, timestamp 저장

4. goal grounding
   - `"소파로 가"` -> 가장 적절한 소파 instance 선택
   - `"소파 앞으로 가"` -> object 기준 approach pose 계산

5. Nav2 연동
   - 계산된 approach pose를 `/goal_pose`로 발행

---

## 추천 단계별 로드맵

### Stage A. Minimal Semantic Navigation

목표:
객체 인식 결과를 map 좌표계에 저장하고,
`"소파로 가"`를 처리한다.

구현:

- detector 추가
- depth/TF projection
- semantic registry 작성
- `label -> pose` goal 변환

출력:

- `"소파로 가"` -> 객체 근처의 reachable pose 이동

### Stage B. Relational Goal Grounding

목표:
객체 관계어를 처리한다.

예:

- `"소파 앞으로 가"`
- `"책상 오른쪽으로 가"`

구현:

- object orientation 또는 surface normal 추정
- relation to pose 변환기 작성
- approach pose 안전성 검증

출력:

- object-relative navigation

### Stage C. Persistent Semantic Memory

목표:
한 번 본 객체를 계속 기억한다.

구현:

- instance tracking
- duplicate merge
- confidence 관리
- 오래된 관측 정리

출력:

- 시야에 없어도 이전 관측을 활용해 navigation 가능

### Stage D. Advanced 3D Semantic Reasoning

목표:
scene graph 또는 더 강한 foundation model 기반 reasoning으로 확장한다.

예:

- `"2층 복도에 있는 파란 소파로 가"`
- `"문 지나서 오른쪽 책상 앞으로 가"`

이 단계는 미래 확장 방향이며,
현재 repo의 즉시 구현 우선순위는 아니다.

---

## 무엇이 "현재 가장 주목받는가"

정리하면 2026년 현재 가장 주목받는 방향은:

**Open-vocabulary perception + semantic memory/scene graph + classical navigation**

이다.

즉 아래 구조가 현재 가장 설득력 있다.

```text
카메라/VLM이 객체를 찾음
-> semantic memory에 저장
-> language query를 객체에 grounding
-> Nav2가 approach pose로 이동
```

반면 아래는 더 미래지향적이지만 아직 주류 표준 구현으로 보긴 어렵다.

```text
3D map 자체를 거대한 foundation model이 바로 이해
-> direct goal grounding
-> direct navigation
```

즉,

- **현재 실용 주류**: semantic layer를 분리해서 구성
- **미래 지향 방향**: 3D map 자체를 더 직접 이해하는 embodied foundation model

---

## 외부 참고 프로젝트 / 자료

아래 자료는 2026-03-11 기준으로 참고 가치가 높은 공개 프로젝트다.

- OneMap
  - GitHub: https://github.com/KTH-RPL/OneMap
  - Site: https://www.finnbusch.com/OneMap/
  - open-vocabulary map 기반 object navigation 방향 참고

- HOV-SG
  - GitHub: https://github.com/hovsg/HOV-SG
  - Site: https://hovsg.github.io/
  - 3D scene graph 기반 language-grounded navigation 참고

- SG-Nav
  - GitHub: https://github.com/bagh2178/SG-Nav
  - Site: https://bagh2178.github.io/SG-Nav/
  - LLM + online 3D scene graph object navigation 참고

- ROS2-NanoOWL
  - GitHub: https://github.com/NVIDIA-AI-IOT/ROS2-NanoOWL
  - ROS2에서 open-vocabulary object detection을 붙일 때 참고

- Isaac ROS Object Detection
  - GitHub: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection
  - Grounding DINO / YOLOv8 / RT-DETR 계열 ROS2 통합 참고

- Nav2
  - Site: https://nav2.org/
  - Docs: https://docs.nav2.org/index.html
  - 최종 goal execution layer 참고

---

## 최종 결론

이 문서의 결론은 아래 한 줄로 요약된다.

> 현재는 geometry map만으로 `"소파로 가"`를 안정적으로 처리하기 어렵고,
> semantic layer를 추가한 뒤 그 결과를 Nav2에 grounding하는 구조가
> 가장 현실적이고 주목받는 구현 방식이다.

즉 이 프로젝트의 다음 방향은

- RTAB-Map / Nav2를 버리는 것이 아니라
- 그 위에 semantic memory 계층을 추가하고
- language command를 object-grounded goal pose로 바꾸는 것

이 되어야 한다.
