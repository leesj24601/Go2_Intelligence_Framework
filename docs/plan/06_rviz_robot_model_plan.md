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
