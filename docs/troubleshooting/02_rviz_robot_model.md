---
tags: [project, rviz, visualization, troubleshooting]
status: resolved
project: go2_intelligence_framework
type: troubleshooting
created: 2026-02-24
---

# RViz2 Go2 Robot Model 트러블슈팅

## 트러블슈팅

## 1. RViz에 TF 축만 보이고 Go2 메쉬가 안 보임

원인:
- `robot_state_publisher` 없음
- `RobotModel` display 없음
- `Description Topic` 미설정

확인:
- `launch/go2_rtabmap.launch.py`에 `robot_state_publisher`가 있는지
- `config/go2_sim.rviz`에 `RobotModel`이 있는지
- RViz에서 `Description Topic = /robot_description`인지

대응:
- `RobotModel` display 추가
- `/robot_description` 명시

---

## 2. RobotModel이 `Error subscribing` 또는 `Topic Error`

원인:
- `Description Topic` 값이 비어 있음
- `/robot_description`을 안 읽고 있음

대응:
- RViz에서 아래 값으로 고정

```text
Description Source: Topic
Description Topic: /robot_description
```

---

## 3. 몸통은 보이는데 다리가 안 보임

원인:
- `/joint_states`가 없음
- `robot_state_publisher`는 URDF는 읽었지만 revolute joint transform을 계산할 입력이 없음

증상:
- `base_link`는 보이는데 다리 링크는 `No transform from ...`
- 또는 몸통만 보이고 다리 메쉬가 없음

대응:
- 임시: `joint_state_publisher` 실행
- 최종: `scripts/go2_sim.py`에서 실제 joint state 퍼블리시

확인:

```bash
ros2 topic echo /joint_states
```

---

## 4. Go2 형상은 보이는데 다리가 안 움직이고 고정 자세로만 평행이동함

원인:
- `joint_state_publisher`가 넣는 고정 `/joint_states`를 쓰는 상태

설명:
- 이 경우 `base_link` 이동/회전은 실제 TF를 따르지만
- 다리 각도는 고정값이라 보행 모션과 동기화되지 않음

대응:
- `joint_state_publisher`를 끄고
- `scripts/go2_sim.py`의 실제 `/joint_states`를 사용

현재 기본값:
- `use_fake_joint_states=false`

---

## 5. `/joint_states`는 나오는데 still TF 축만 너무 많이 보여서 형상이 안 보임

원인:
- RViz `TF` display가 좌표축과 프레임 이름을 과하게 표시

대응:
- `TF` display 끄기
- 또는 `Show Axes`, `Show Names` 끄기
- RobotModel만 먼저 확인

현재 `config/go2_sim.rviz`는 기본 TF를 꺼둔 상태다.

---

## 6. `go2_description.urdf.xacro`를 쓰면 잘 안 보이고 `go2_description.urdf`는 보임

원인:
- 이 패키지에서는 RViz visual 쪽에서 `urdf`가 더 안정적으로 동작했다
- 특히 standalone 검증 과정에서 `urdf` 쪽이 더 예측 가능했다

결론:
- 구현 구조는 ROS 표준대로 `xacro/URDF -> robot_state_publisher`
- 하지만 기본 파일은 `go2_description.urdf`를 사용

즉:
- 패턴은 표준
- 실제 파일 선택은 현 프로젝트에서 검증된 쪽

---

## 7. `camera_link earlier than all the data in transform cache`

원인:
- `/scan` 또는 카메라 메시지 타임스탬프와 TF 캐시 타이밍 차이

영향:
- 주로 이미지/레이저 표시 드롭 경고
- RobotModel이 안 뜨는 직접 원인은 아님

해석:
- 이 메시지가 보여도 Go2 RobotModel 문제와는 분리해서 봐야 한다

---

## 7.5 RViz `Fixed Frame`과 `/scan` 끊김 해석

### `Fixed Frame`이란?

RViz의 `Fixed Frame`은
모든 토픽을 **최종적으로 어느 좌표계 기준으로 그릴지** 정하는 기준 프레임이다.

예를 들어:
- `/scan`의 `frame_id`가 `camera_link`
- RViz `Fixed Frame`이 `map`

이면 RViz는 `/scan`을 바로 그리지 않고,
`map -> odom -> base_link -> camera_link`
TF 체인을 따라 `camera_link` 데이터를 `map` 기준으로 변환해서 그린다.

즉:
- 서로 다른 프레임의 토픽을 한 화면에 겹쳐 볼 수 있게 해 주는 기준 좌표계가 `Fixed Frame`
- `Fixed Frame`이 바뀌면 같은 `/scan`도 화면에서 보이는 방식이 달라질 수 있음

### 왜 `map` 기준에서는 더 끊겨 보일 수 있나?

핵심은 **변환 단계 수 자체**보다 아래 3가지다.

1. 해당 시점의 TF가 존재하는가
2. TF가 충분히 자주 갱신되는가
3. 메시지 타임스탬프와 TF 타이밍이 잘 맞는가

즉:
- TF 체인이 길다고 해서 그것만으로 느려지는 경우는 드물다
- 대신 중간 프레임 중 하나라도 늦게 갱신되거나
- `/scan` 시각에 해당하는 TF를 못 찾거나
- `map -> odom` 보정이 상대적으로 천천히 갱신되면
  RViz에서는 레이저가 뚝뚝 끊기거나 순간이동하듯 보일 수 있다

특히 이 프로젝트에서는:
- `/scan`은 `camera_link` 프레임
- RViz `Fixed Frame`은 기본적으로 `map`
- `odom -> base_link`는 시뮬/오도메트리
- `map -> odom`은 RTAB-Map localization 결과

이 구조라서 `/scan` 자체 주파수가 충분해도
`map` 기준에서는 localization 보정이 보이며 덜 부드럽게 느껴질 수 있다.

### 어떤 `Fixed Frame`을 언제 쓰면 좋은가?

- `map`
  - 목적: localization이 맵과 잘 맞는지 확인
  - 장점: 빨간 `/scan` 선과 맵의 정합 상태를 보기 좋음
  - 단점: `map -> odom` 보정이 보이면 scan이 약간 튀어 보일 수 있음

- `odom`
  - 목적: 센서/로봇 움직임이 얼마나 부드럽게 보이는지 확인
  - 장점: `map` 보정 영향이 줄어 scan이 더 부드럽게 보이는 경우가 많음
  - 단점: 장기적으로는 맵 기준 드리프트가 보일 수 있음

- `base_link` 또는 `camera_link`
  - 목적: 센서 원본 데이터가 자체적으로 끊기는지 확인
  - 장점: `/scan` 자체 품질 점검에 유리
  - 단점: 맵 정합 확인에는 부적합

### 실전 디버깅 팁

레이저가 끊겨 보일 때는 아래 순서로 보면 원인을 빠르게 분리할 수 있다.

1. `Fixed Frame = base_link` 또는 `camera_link`
   - 여기서도 끊기면 `/scan` 자체 주기나 센서 입력을 의심
2. `Fixed Frame = odom`
   - 여기서 부드러워지면 `map -> odom` 보정 영향 가능성 큼
3. `Fixed Frame = map`
   - 이 화면은 센서 부드러움 확인용보다는 localization 정합 확인용으로 해석

즉:
- `/scan` 품질 확인은 `odom`/`base_link`
- localization 정합 확인은 `map`

으로 역할을 나눠 보는 것이 가장 실용적이다.

---

## 설계 결론

이번 이슈에서 가장 효과적이었던 방법은 아래 조합이었다.

1. `robot_state_publisher`로 Go2 URDF 공급
2. RViz `RobotModel`이 `/robot_description` 사용
3. 초기 디버그는 `joint_state_publisher`로 빠르게 확인
4. 최종은 Isaac Sim 실제 articulation 값을 `/joint_states`로 퍼블리시

즉 커뮤니티 표준 구조를 따르되,
모델 파일은 실제로 RViz에서 검증된 `go2_description.urdf`를 기본으로 쓰고,
관절은 fake publisher가 아니라 실제 시뮬레이션 값을 연결하는 쪽이 최종 정답이었다.

---

## 관련 파일

- `launch/go2_rtabmap.launch.py`
- `config/go2_sim.rviz`
- `scripts/go2_sim.py`
