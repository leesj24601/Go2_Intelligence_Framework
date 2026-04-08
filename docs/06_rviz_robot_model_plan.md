---
tags: [project, rviz, visualization]
status: completed
project: go2_intelligence_framework
type: implementation-plan
created: 2026-02-24
---

# RViz2 Go2 Robot Model: 구현 및 트러블슈팅

## Goal
RViz2에서 `map -> odom -> base_link -> camera_link -> camera_optical_frame` TF만 보이는 상태를 넘어서,
실제 Go2 로봇 형상을 표시하고, 최종적으로는 Isaac Sim의 실제 다리 관절 움직임까지 동기화한다.

---

## 최종 구현 결과

현재 적용된 구조는 아래와 같다.

```text
go2_description.urdf
        |
        v
robot_state_publisher
        |
        +--> /robot_description
        +--> /tf, /tf_static (base_link 이하 링크들)

Isaac Sim go2_sim.py
        |
        +--> /odom
        +--> /tf          (odom -> base_link)
        +--> /joint_states  (실제 articulation joint 값)
        +--> /camera/*, /imu/data

RViz2
  - RobotModel  (/robot_description 사용)
  - Map
  - PointCloud2
  - LaserScan
```

핵심은 다음 3가지다.

1. `robot_state_publisher`가 Go2 URDF를 공급해야 함
2. RViz `RobotModel`이 `/robot_description`을 읽어야 함
3. 다리까지 움직이게 하려면 `/joint_states`를 Isaac Sim 실제 값으로 퍼블리시해야 함

---

## 실제 구현 방법

## 1. launch에서 robot_description 공급

파일: `launch/go2_rtabmap.launch.py`

적용 방식:
- `robot_state_publisher` 추가
- `go2_description` 패키지의 URDF를 launch에서 로드
- `robot_description`는 `xacro`/URDF 공용 패턴으로 처리

현재 기본 설정:
- `description_package=go2_description`
- `description_file=go2_description.urdf`

주의:
- 이 패키지에서는 `go2_description.urdf`가 RViz visual 표시가 더 안정적이었다
- 구조는 표준 방식(`xacro/URDF -> robot_state_publisher`)으로 유지하되, 기본 파일은 `urdf`를 사용
- 필요하면 launch 인자로 `go2_description.urdf.xacro`를 줄 수 있게 열어두었다

---

## xacro 방식 vs URDF 직접 읽기 방식

`robot_description`을 공급하는 방법은 크게 2가지다.

### 1. xacro 방식

예:

```python
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([FindPackageShare("go2_description"), "urdf", "go2_description.urdf.xacro"]),
])
```

장점:
- ROS 커뮤니티에서 가장 흔히 쓰는 표준 패턴
- 센서 유무, prefix, 실로봇/시뮬 분기 같은 조건부 구성이 쉬움
- 여러 로봇 변형을 하나의 템플릿으로 관리하기 좋음

단점:
- xacro include, 매크로, path 해석이 꼬이면 디버깅이 길어질 수 있음
- 실제 렌더링 문제인지 xacro 변환 문제인지 분리해서 봐야 할 때가 있음

### 2. URDF 직접 읽기 방식

예:

```python
robot_description = Path("/path/to/go2_description.urdf").read_text()
```

장점:
- 가장 단순해서 디버깅이 쉬움
- 현재 RViz에 들어간 최종 XML을 바로 확인할 수 있음
- xacro 변환 문제를 배제하고 원인을 좁히기 좋음

단점:
- 조건부 구성, prefix, 모델 변형 관리가 불편함
- 파일 하나를 고정적으로 읽는 방식이라 확장성이 떨어짐

### 현재 프로젝트에서의 선택

우리는 **구조는 xacro/URDF 공용 표준 패턴을 유지**하고, **기본 파일은 `go2_description.urdf`를 사용**한다.

즉 현재 방식은 아래와 같다.

- launch 레벨 구현 패턴: `Command + robot_state_publisher`
- 기본 description 파일: `go2_description.urdf`
- 필요 시 launch argument로 `go2_description.urdf.xacro`로 바꿔 실행 가능

이렇게 한 이유:
- ROS 개발 관행상 `xacro -> robot_state_publisher` 패턴은 유지하는 게 맞음
- 하지만 실제 이 프로젝트의 `go2_description` 패키지에서는 RViz visual 검증 결과 `urdf`가 더 안정적이었음
- 따라서 표준성과 실용성을 같이 가져가는 절충안을 택함

쉽게 말하면:

- `xacro 방식`: 부품 파일들을 다시 조립해서 최종 Go2 모델을 만드는 방식
- `URDF 직접 읽기`: 이미 조립 완료된 Go2 완성품을 그대로 읽어 쓰는 방식

우리 프로젝트는 `go2_description` 패키지 안에
- 부품용 xacro 파일들도 있고
- 이미 조립된 `go2_description.urdf`도 같이 있다

그래서 현재 기본값은:
- 굳이 xacro로 다시 조립하지 않고
- 이미 완성된 `go2_description.urdf`를 바로 읽어 쓰는 방식

즉 한 줄로 말하면:
- 부품도 따로 있음
- 완성품도 이미 있음
- 우리는 지금 완성된 Go2를 그냥 불러오는 쪽

---

## 2. RViz에 RobotModel display 추가

파일: `config/go2_sim.rviz`

적용 내용:
- `rviz_default_plugins/RobotModel` 추가
- `Description Source: Topic`
- `Description Topic: /robot_description`

추가 조정:
- 기본 `TF` display는 꺼둠
- 이유: TF 축과 프레임 이름이 너무 강하게 보여서 실제 메쉬가 묻히기 쉬웠음

---

## 3. fake joint state 대신 Isaac Sim 실제 joint state 발행

파일: `scripts/go2_sim.py`

초기에는 `joint_state_publisher`로 고정 자세 `/joint_states`를 넣어 RobotModel을 띄웠다.
이 방식은 형상 표시 자체에는 충분하지만, 실제 보행 모션과는 동기화되지 않는다.

그래서 최종적으로 아래 방식으로 변경했다.

- `env.unwrapped.scene["robot"]`에서 articulation handle 획득
- `robot.joint_names` 사용
- `robot.data.joint_pos[0]`, `robot.data.joint_vel[0]`를 매 프레임 읽음
- `sensor_msgs/JointState`를 `/joint_states`로 퍼블리시

즉 지금은:
- 위치/자세: 실제 `odom -> base_link`
- 다리 관절: 실제 Isaac Sim joint 값

으로 RViz에서 보인다.

---

## 4. fake joint_state_publisher는 기본 비활성화

파일: `launch/go2_rtabmap.launch.py`

현재 launch에는 디버그용 `joint_state_publisher`가 남아 있지만 기본값은 꺼져 있다.

```text
use_fake_joint_states:=false
```

이유:
- 실사용 시 `go2_sim.py`가 실제 `/joint_states`를 발행하므로 중복되면 안 됨
- 다만 Isaac Sim 없이 RViz에서 고정 자세만 확인하고 싶을 때는 fallback으로 쓸 수 있음

---

## 실행 방법

## 1. Isaac Sim 실행

```bash
/home/cvr/anaconda3/envs/lab/bin/python /home/cvr/Desktop/sj/go2_intelligence_framework/scripts/go2_sim.py
```

## 2. RTAB-Map launch 실행

```bash
source /opt/ros/humble/setup.bash
source /home/cvr/Desktop/sj/go2_ws/install/setup.bash
ros2 launch /home/cvr/Desktop/sj/go2_intelligence_framework/launch/go2_rtabmap.launch.py
```

## 3. RViz 실행

```bash
rviz2 -d /home/cvr/Desktop/sj/go2_intelligence_framework/config/go2_sim.rviz
```

---

## 정상 동작 기준

아래가 모두 맞으면 정상이다.

- RViz `RobotModel` 상태가 `OK`
- Go2 메쉬가 보임
- 로봇이 이동할 때 RViz의 Go2도 같이 이동함
- 보행할 때 다리 관절도 같이 움직임
- `/joint_states`가 실제 값으로 발행됨

확인 명령:

```bash
ros2 topic echo /joint_states
```

여기서 걷는 중에 값이 계속 바뀌면 실제 관절 동기화가 되는 상태다.

---

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
