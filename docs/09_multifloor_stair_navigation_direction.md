---
tags: [project, research, stair-navigation]
status: research
project: go2_intelligence_framework
type: research-notes
created: 2026-02-24
---

# 09. Go2 복층 / 계단 Navigation Direction (2026)

## Goal

현재 프로젝트는 단층 환경에서

- RTAB-Map 기반 맵 생성 / localization
- Nav2 기반 Goal Pose 자율주행
- Go2 RL locomotion과 `/cmd_vel` 연동

까지는 이미 구현 방향이 정리되어 있다.

다음 단계에서 검토할 주제는,

- 건물 내부에서 층과 층이 직접 연결된 계단을 Go2가 오르내리기
- 복층 환경에서 map / localization / navigation을 어떻게 운영할지
- 하나의 연결된 맵으로 갈지, 층별 맵으로 나눌지

이다.

이 문서는 그에 대한 개념 정리와,
**2026년 기준으로 가장 현실적인 Go2 복층 내비게이션 구조**를 정리한 작업 문서다.

---

## 핵심 결론

결론은 단순하다.

### 1. 맵 생성은 하나의 연결된 3D 글로벌 맵으로 가는 것이 맞다

Go2가 실제로 계단을 오르내릴 수 있고,
FAST-LIO 같은 6DoF LiDAR-IMU odometry / SLAM 또는 RTAB-Map 기반 3D mapping이 안정적으로 동작한다면,

- 1층
- 계단
- 2층

을 **한 번에 연속적으로 매핑하여 하나의 연결된 3D 글로벌 맵**으로 만들 수 있다.

즉 사용자가 조사한
"1층에서 시작해서 계단을 타고 2층으로 올라가며 하나의 맵으로 이어 붙인다"
는 방향은 맞다.

### 2. 하지만 내비게이션 운용은 stock Nav2 하나로 끝나지 않는다

여기서 중요한 구분이 있다.

**하나의 연결된 3D 글로벌 맵을 만들 수 있다는 것**과  
**stock Nav2가 그 맵 위에서 계단까지 자연스럽게 주행 경로로 처리할 수 있다는 것**은 같은 말이 아니다.

Nav2는 기본적으로 `nav2_costmap_2d` 기반이다.
즉 3D 맵이 있어도 기본 planner / controller는 여전히 2D 평면 내비게이션 중심이다.

따라서 실질적인 운영 구조는 아래처럼 가는 것이 맞다.

- **맵/상태추정**: 하나의 연결된 3D 글로벌 맵
- **층 내부 이동**: Nav2
- **계단 구간**: Stair Supervisor가 reference를 만들고, Go2 locomotion이 실제 보행 수행

즉 정리하면:

**맵은 하나로 연결하고, 주행 실행은 층별 Nav2 + Stair Supervisor + Locomotion으로 나눈다.**

---

## 역할 분리

현재 논의 기준으로는 시스템을 아래처럼 나누는 것이 가장 명확하다.

### 1. State Estimation / Mapping

역할:

- LiDAR / Camera / IMU로 현재 pose 추정
- 1층-계단-2층을 잇는 연속 3D 글로벌 맵 생성
- 상위 계층에 현재 위치, 자세, 속도 제공

대표 후보:

- FAST-LIO
- RTAB-Map
- 그 외 6DoF LiDAR-IMU / Visual-Inertial SLAM

이 계층은
**"로봇이 지금 3D 공간에서 어디 있는가"**를 담당한다.

### 2. Mission / Task Layer

역할:

- 최종 목적지 해석
- 목적지가 몇 층인지 판단
- 어느 계단이나 포털을 탈지 결정

예:

- `"2층 실험실로 가"`
- `"1층으로 내려가"`

이 계층은
**"어디로 가야 하는가"**를 담당한다.

### 3. Route Graph / Multi-floor Planner

역할:

- 복층 경로를 구간 단위로 나눔
- 같은 층 평지 구간과 계단 구간을 구분
- `1F_lobby -> stairs_bottom -> stairs_top -> 2F_goal` 같은 상위 경로 생성

이 계층은
**"어떤 구간 순서로 이동할 것인가"**를 담당한다.

### 4. Floor Navigation Layer

역할:

- 같은 층 평지 / 복도 / 로비 구간 주행
- `stairs_bottom`, `2F_goal` 같은 pose까지 이동

현재 프로젝트에서는 여기에 **Nav2**가 들어간다.

이 계층은
**"같은 층 안에서 어떻게 갈 것인가"**를 담당한다.

### 5. Stair Supervisor Layer

이 계층이 이번 논의에서 가장 중요하다.

역할:

- 계단 진입 여부 판단
- 계단 중심축 방향으로 yaw 정렬
- 올라갈지 / 내려갈지 결정
- 계단 구간에서 사용할 속도 reference 생성
- progress / abort / timeout / success 판정

즉 질문했던
**"계단에서 어느 방향으로 오를지, 속도 명령을 누가 주느냐"**
의 답이 바로 이 계층이다.

이 계층은 보통 아래와 같은 명령을 만든다.

- `vx`
- `wz`
- heading reference
- body posture reference
- stair mode on/off

즉 **계단 전용 중간 제어기**다.

### 6. Locomotion Layer

역할:

- 상위에서 받은 velocity / heading / mode 명령을 실제 보행으로 변환
- 평지면 평지 gait
- 계단이면 stair-compatible gait
- 넘어지지 않도록 자세 안정화

대표 후보:

- unified RL policy
- Unitree 기본 제어기
- 커스텀 locomotion controller

이 계층은
**"실제로 어떻게 걷는가"**를 담당한다.

### 7. Low-level Robot Control

역할:

- 관절 명령
- 모터 제어
- 접지와 균형 유지

이 계층은 최종적으로 실제 Go2 하드웨어를 움직인다.

---

## 한 번에 복층 맵이 만들어지는 과정

복층 연속 매핑은 아래처럼 이해하면 된다.

### 1. 1층 매핑

로봇이 1층을 주행하면,
SLAM은 1층 구조를 기준 좌표계에서 계속 누적한다.

### 2. 계단 진입

로봇이 계단을 오르기 시작하면,
하체 제어기는 계단을 오를 수 있도록 다리 동작을 안정화한다.

동시에 SLAM은

- 전진
- 상승
- body orientation 변화

를 반영해 trajectory를 계속 적분한다.

### 3. 계단 구간 매핑

이 구간에서 SLAM은
"로봇이 X로 전진하면서 Z로 상승한다"는 사실을 반영해
계단 형상을 포함한 3D 구조를 연속적으로 맵에 추가한다.

### 4. 2층 진입

로봇이 2층 landing에 도달하면,
SLAM은 그 위치를 같은 글로벌 좌표계에서 이어서 누적한다.

그 결과:

- 1층
- 계단
- 2층

이 하나의 map frame 안에서 연결된 **single 3D global map**이 된다.

---

## 왜 "연결된 3D 맵"과 "Nav2 운용"을 구분해야 하는가

이 부분이 가장 중요하다.

### 연결된 3D 맵이 필요한 이유

복층을 따로따로 맵핑하면,

- 1층과 2층 사이의 실제 연결 관계가 map 차원에서 약해지고
- 계단이 공간적으로 끊어진 구조처럼 보일 수 있고
- 상위 planner가 전체 건물 구조를 이해하기 어려워진다

반대로 하나의 연결된 3D 맵이면

- 계단의 실제 위치와 형상 보존
- 층 간 연결성 유지
- 장기적으로 3D planner / semantic graph 확장 용이

라는 장점이 있다.

### 그런데도 Nav2를 층별로 운영하는 이유

Nav2는 공식적으로 2D costmap 기반이므로,
연결된 3D 맵이 있더라도 계단을 기본 local/global planner가 그대로 통과 가능한 free-space로 처리하는 것은 별도 문제다.

즉 실제 실행은 보통 이렇게 나뉜다.

1. 같은 층 내부 이동
   - Nav2 사용

2. 계단 진입
   - 정렬 / 속도 저감 / mode 전환

3. 계단 오르내리기
   - Stair Supervisor + Go2 Locomotion

4. 다음 층 landing 이후
   - 다시 Nav2 사용

따라서
**맵 생성은 하나로 연결하되, 실행 로직은 구간별로 나누는 것이 현실적이다.**

---

## 추천 아키텍처

### 전체 구조

```text
[1] Mission / Task Layer
    - 최종 목적지 결정
    - 목표 층 판단

                |
                v

[2] Route Graph / Multi-floor Planner
    - 1F -> stairs_bottom -> stairs_top -> 2F_goal
    - 경로를 구간 단위로 분해

                |
        +-------+-------+
        |               |
        v               v

[3A] Floor Navigation Layer
     - Nav2
     - 같은 층 평지 구간 이동

[3B] Stair Supervisor Layer
     - 계단 입구 정렬
     - 진행 방향 결정
     - 속도 / heading reference 생성
     - 성공 / 실패 판정

                |
                v

[4] Locomotion Layer
    - Go2 unified policy 또는 보행 제어기
    - 받은 reference를 실제 보행으로 변환

                |
                v

[5] Low-level Robot Control
    - joint command
    - motor control
```

핵심은 이것이다.

- **3D 위치 추정과 글로벌 맵**은 SLAM이 담당
- **복층 구간 분해**는 route graph가 담당
- **층 내부 평지 이동**은 Nav2가 담당
- **계단 방향 / 속도 reference 생성**은 stair supervisor가 담당
- **실제 발 디딤과 안정화**는 locomotion이 담당

### 상태 추정 계층은 별도로 계속 돈다

위 구조와 별개로 아래 계층은 항상 함께 동작한다.

```text
[State Estimation / Mapping]
- FAST-LIO / RTAB-Map / IMU / LiDAR / Camera
- 현재 pose, 속도, 자세 추정
- 연결된 3D global map 생성
```

즉 전체 시스템은
`State Estimation`을 바탕으로
`Mission -> Route -> Nav2/Stair Supervisor -> Locomotion`
순서로 이어진다고 보는 것이 맞다.

---

## Elevation Map / Traversability의 역할

복층 / 계단 navigation에서
`Elevation Map`과 `Traversability`는 **안 쓰는 것이 아니라 적극적으로 쓰는 것이 맞다.**

다만 이 둘은 보통 시스템 전체를 단독으로 해결하는 주인공이라기보다,
**상위 planner와 stair supervisor를 더 똑똑하게 만드는 보강 레이어**로 이해하는 편이 정확하다.

### 1. Elevation Map

Elevation Map은
로봇 주변 지형의 높이 변화를 표현하는 지도다.

즉 단순히

- 갈 수 있음 / 없음

만 보는 것이 아니라,

- 바닥 높이
- 경사
- 턱 높이
- 계단 단차

같은 정보를 유지한다.

복층 계단 시나리오에서 Elevation Map은 아래처럼 유용하다.

- 계단 입구와 계단 축 추정
- landing과 stair body 구간 구분
- 현재 지면이 평지인지 경사인지 단차인지 판단
- 향후 terrain-aware planner 확장 기반

### 2. Traversability

Traversability는
"이 지형이 로봇에게 얼마나 지나가기 쉬운가"
를 수치화한 레이어다.

즉 계단이나 턱을 단순히 장애물로 막아버리는 대신,

- 쉽게 통과 가능
- 조심해서 통과 가능
- 사실상 불가능

처럼 해석하게 해준다.

복층 / 계단 시나리오에서는 예를 들어 아래 판단에 도움이 된다.

- 이 계단은 Go2가 오를 수 있는 단차인가
- 이 진입 방향이 가장 안정적인가
- 왼쪽 계단과 오른쪽 계단 중 어느 쪽이 더 안전한가
- 현재 위치에서 stair entry에 접근 가능한가

### 3. 우리 구조에서 어디에 들어가는가

현재 추천 구조에서 이 둘은 아래처럼 들어간다.

```text
Sensors
  -> State Estimation / 3D Mapping
  -> Elevation Map / Traversability Layer
  -> Mission / Route Graph / Stair Supervisor
  -> Nav2 / Locomotion
```

즉:

- `State Estimation / 3D Mapping`
  - 원본 3D 구조와 pose를 제공

- `Elevation Map / Traversability`
  - 그 구조를 "지형 정보"로 해석

- `Route Graph / Stair Supervisor`
  - 그 해석 결과를 바탕으로 계단 선택, 진입 방향, 속도 전략 결정

### 4. 왜 주 시스템을 대체하진 않는가

중요한 점은,
Elevation Map과 Traversability가 있어도 여전히 아래 질문은 남는다는 것이다.

- 목적지가 몇 층인가
- 어느 계단을 탈 것인가
- 계단에서 어느 방향으로 정렬할 것인가
- 실제로 얼마 속도로 전진할 것인가
- 실제 발 디딤을 어떻게 할 것인가

즉 이 둘은 매우 중요하지만,

- `Mission / Route Graph`
- `Stair Supervisor`
- `Locomotion`

을 대체하진 않는다.

### 5. 현재 프로젝트에서의 위치

현재 프로젝트 기준으로는
Elevation Map / Traversability를 아래처럼 보는 것이 적절하다.

- 지금 당장 전체 구조를 갈아엎는 대체재는 아님
- 그러나 계단 판단과 험지 이해를 위해 매우 유용한 핵심 보강 레이어
- 장기적으로 3D terrain-aware planner로 확장할 때 중요한 기반

즉 결론은:

**우리는 Elevation Map과 Traversability를 쓴다.**

다만 그것만으로 모든 문제가 해결되는 것은 아니며,
현재 구조에서는
**Route Graph와 Stair Supervisor를 강화하는 핵심 입력 레이어**로 쓰는 것이 맞다.

---

## 우리가 실제로 선택할 구조

현재 프로젝트 기준으로는 아래 구조가 가장 합리적이다.

### A. 맵 생성

- 1층, 계단, 2층을 **한 번에 연속 3D 맵으로 생성**

### B. 경로 표현

- 상위에서는 route graph 또는 portal graph 사용
- 예:
  - `1F_lobby`
  - `stairs_bottom`
  - `stairs_top`
  - `2F_lobby`

### C. 주행 실행

- `1F_lobby -> stairs_bottom`
  - Nav2

- `stairs_bottom -> stairs_top`
  - Stair Supervisor + Locomotion

- `stairs_top -> 2F_goal`
  - Nav2

즉,
**연결된 글로벌 맵 위에 topological route를 얹고,
구간별 실행기는 Nav2와 Stair Supervisor / Locomotion으로 나눈다.**

---

## 왜 층별 2D map이 여전히 등장하는가

여기서 헷갈리기 쉬운 부분이 있다.

우리가 "층별 2D map"을 말할 때,
그것은 **복층 전체를 층별로 따로 SLAM한다**는 뜻이 아니라,
대개 **Nav2가 실제로 쓰기 쉬운 2D 운용 레이어를 층별로 본다**는 의미다.

즉,

- 글로벌 기준: 하나의 연결된 3D map
- 실행 기준: 층별 2D traversable region 또는 층별 planner context

로 나뉘는 것이다.

그래서 결론은
"맵도 따로 만들고 내비도 따로 만든다"가 아니라,

**맵은 연결하고, 내비게이션 레이어만 층별로 분리한다**

가 더 정확하다.

---

## 추천 상태기 / BT 흐름

```text
1. 최종 목표가 다른 층인지 판단
2. Route graph에서 stair portal 계산
3. Nav2로 stairs_bottom까지 이동
4. Stair Supervisor가 계단 중심축으로 정렬
5. Stair Supervisor가 전진 / yaw 속도 reference 생성
6. Locomotion policy가 실제 계단 보행 수행
7. Stair Supervisor가 progress / abort / success 판정
8. stairs_top 도달 확인
9. Nav2로 다시 전환
10. 최종 목표까지 이동
```

BT 스타일로 쓰면 대략 아래와 같다.

```text
Sequence
  - ComputeMultiFloorRoute
  - NavigateToPose(stairs_bottom)
  - AlignToStairs
  - RunStairSupervisor
  - ExecuteLocomotion
  - MonitorStairTraversal
  - ResumeNav2
  - NavigateToPose(final_goal)
```

핵심은
`StartStairTraversal` 하나로 모든 것을 뭉뚱그리기보다,

- 방향 정렬
- reference 생성
- locomotion 실행
- 성공 / 실패 판정

이 네 가지를 분리해서 보는 편이 정확하다는 점이다.

---

## 현재 프로젝트에 맞는 1차 구현안

### Phase 1

- FAST-LIO 또는 동급 6DoF odometry/SLAM 검토
- Go2로 1층-계단-2층을 **한 번에 연속 맵핑**

### Phase 2

- stair entry / exit waypoint 정의
- route graph 데이터 모델 추가

### Phase 3

- stair supervisor 인터페이스 정의
- 예:
  - `AlignToStairs`
  - `RunStairSupervisor`
  - `MonitorStairTraversal`

### Phase 4

- Nav2 BT에 custom node 추가
- `AlignToStairs`
- `RunStairSupervisor`
- `ExecuteLocomotion`
- `ResumeNav2`

### Phase 5

- 복층 목적지 명령 처리
- 예:
  - `"2층 복도로 가"`
  - `"위층 실험실로 가"`
  - `"1층으로 내려가"`

---

## 지금 단계에서의 판단

현재 질문
"각 층 맵을 따로 만드는가, 아니면 한 번에 연결된 맵을 만드는가?"
에 대한 답은 아래와 같다.

### 답

- **맵은 한 번에 연결된 3D 글로벌 맵으로 만든다**
- **하지만 자율주행 실행은 층별 Nav2 + Stair Supervisor + Locomotion으로 나눈다**

즉,

**연결된 맵 + 역할이 분리된 실행기**

가 현재 가장 설득력 있는 구조다.

---

## 관련 링크

- Nav2 Mapping / Localization
  - https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- Nav2 Behavior Trees
  - https://docs.nav2.org/behavior_trees/
- Nav2 BT Navigator
  - https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
- Nav2 Route Server
  - https://docs.nav2.org/configuration/packages/configuring-route-server.html
- Nav2 Costmaps
  - https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- RTAB-Map ROS 2
  - https://github.com/introlab/rtabmap_ros
- FAST-LIO
  - https://github.com/hku-mars/FAST_LIO
- Unitree ROS 2
  - https://github.com/unitreerobotics/unitree_ros2
- Unitree sports services
  - https://support.unitree.com/home/en/developer/sports_services
- vox_nav
  - https://github.com/NMBURobotics/vox_nav

---

## 다음 문서 후보

이 문서 다음 단계로는 아래 두 문서가 자연스럽다.

1. **Go2 Stair Traversal BT / State Machine 상세 설계**
   - action / topic / failure condition / recovery flow 정의

2. **복층 route graph / portal graph 데이터 모델 설계**
   - stair entry, stair exit, floor id, traversal type 정의
