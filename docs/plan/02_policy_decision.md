---
tags: [project, decision, rl-policy]
status: completed
project: go2_intelligence_framework
type: architecture-decision
created: 2026-02-24
---

# RL 정책 전환: IsaacLab 공식 → unitree_rl_lab

**결정일**: 2026-02-24 | **상태**: ✅ 통합 완료

---

## 한눈에 보기

| 항목 | 기존 (IsaacLab 공식) | 변경 (unitree_rl_lab) |
|---|---|---|
| 소스 | IsaacLab 서버 자동 다운로드 | Unitree 공식 로컬 학습 |
| 태스크 | `Isaac-Velocity-Rough-Unitree-Go2-v0` | `Unitree-Go2-Velocity` |
| 시뮬레이터 | Isaac Lab / PhysX | Isaac Lab 2.3.0 / PhysX (동일) |
| actor obs | ~235-dim (height_scan 포함) | **45-dim** |
| 실로봇 배포 | ❌ 없음 | ✅ `deploy/robots/go2/` 포함 |

---

## 전환 이유

1. **실로봇 배포**: 최종 목표는 실제 Go2에 Nav2 자율주행을 올리는 것.
   IsaacLab 공식 정책은 실로봇 배포 파이프라인이 없고, unitree_rl_lab은 `deploy/` 코드가 포함됨.

2. **Sim-to-Sim 갭 없음**: 둘 다 Isaac Lab / PhysX 기반이므로 별도 검증 불필요. 체크포인트만 교체하면 됨.

3. **동일 기술 스택**: RSL-RL 기반이어서 기존 `go2_sim.py` 파이프라인 변경 최소화.

---

## unitree_rl_lab 설정

### 경로
```
/home/cvr/Desktop/sj/unitree_rl_lab/
```

### USD 모델 경로 (설치 시 1회 설정)
- 파일: `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`
- `UNITREE_MODEL_DIR = "/home/cvr/Desktop/sj/unitree_model"`

### 학습 명령
```bash
cd /home/cvr/Desktop/sj/unitree_rl_lab
python scripts/rsl_rl/train.py --headless --task Unitree-Go2-Velocity
```

### 학습 재개
```bash
python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity \
  --headless \
  --resume \
  --load_run <세션명>    # 예: 2026-02-24_14-26-01
  --checkpoint <모델명>  # 예: model_41600.pt
```

### 검증 (play)
```bash
./unitree_rl_lab.sh -p --task Unitree-Go2-Velocity
```

### 학습 로그 경로
```
logs/rsl_rl/unitree_go2_velocity/
├── 2026-02-23_17-47-48/
├── 2026-02-24_12-51-50/
├── 2026-02-24_12-56-05/
└── 2026-02-24_14-26-01/   ← 현재 활성 세션 (학습 진행 중)
```

---

## go2_sim.py 통합 구현 ✅

변경된 파일: `scripts/go2_sim.py`, `scripts/my_slam_env.py`

### 1. 체크포인트 자동 탐색 (`go2_sim.py`)

학습 진행 중에도 실행 시마다 최신 체크포인트를 자동 선택:

```python
_log_dir = "/home/cvr/Desktop/sj/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity"
_sessions = sorted(glob.glob(os.path.join(_log_dir, "*")))   # 날짜순
_pts = sorted(..., key=lambda p: int(re.search(r"model_(\d+)\.pt", p).group(1)))
resume_path = _pts[-1]  # 번호 최신
```

### 2. obs space 수정 (`my_slam_env.py`)

`UnitreeGo2RoughEnvCfg`의 기본 obs에서 unitree_rl_lab 정책(45-dim)에 맞게 오버라이드:

| obs 항목 | 기본 | unitree_rl_lab | 조치 |
|---|---|---|---|
| `base_lin_vel` (3) | ✅ | ❌ | 제거 |
| `height_scan` (~187) | ✅ | ❌ | 제거 |
| `base_ang_vel` (3) | scale 없음 | scale=0.2 | 재정의 |
| `joint_vel` (12) | scale 없음 | scale=0.05 | 재정의 |
| `projected_gravity`, `velocity_commands`, `joint_pos_rel`, `last_action` | ✅ | ✅ | 유지 |
| **합계** | **~235-dim** | **45-dim** | |

### 3. 기본 자세와 정책 입출력의 관계

Go2 정책은 관절값을 절대 좌표계에서 직접 다루지 않는다.
학습과 추론 모두 **"기본 자세(default joint pose)를 원점으로 한 상대 표현"** 을 사용한다.

#### reset / respawn 시점

- 로봇은 먼저 Go2의 기본 관절 자세로 초기화된다.
- 이 기본값은 Isaac Lab의 `UNITREE_GO2_CFG.init_state.joint_pos`에 정의되어 있다.
- 따라서 정책이 아직 큰 액션을 내지 않아도, 리셋 직후 로봇은 이미 서 있을 수 있는 자세에서 시작한다.

#### 정책 입력

관절 관련 입력은 절대 관절값이 아니라 다음 상대값이다.

```python
joint_pos_rel = current_joint_pos - default_joint_pos
```

즉 정책은 "현재 관절이 기본 자세에서 얼마나 벗어났는가"를 입력으로 본다.
속도 명령(`velocity_commands`)도 함께 들어가므로, 정책은
"기본 자세 근처에서 정지 유지할지"
또는
"기본 자세를 기준으로 어떤 보행 패턴을 만들지"
를 결정한다.

#### 정책 출력

정책 출력 역시 최종 절대 관절 목표가 아니라, 기본 자세에 더해질 보정값으로 해석된다.
Isaac Lab의 `JointPositionActionCfg(..., use_default_offset=True)` 설정 때문에
액션 적용 시 내부적으로 default joint pose가 offset으로 사용된다.

개념적으로는 다음과 같다.

```python
target_joint_pos = default_joint_pos + action_scale * policy_output
```

따라서 정책 출력이 0에 가까우면 로봇은 기본 자세를 유지하려고 한다.

#### standing 이 잘 되는 이유

정지 상태에서 Go2가 잘 서 있는 이유는 두 요소가 함께 작동하기 때문이다.

1. reset 시 이미 서 있기 좋은 기본 자세로 초기화된다.
2. 정책이 그 기본 자세 주변에서 균형을 유지하도록 학습되어 있다.

즉 standing 동작은 "기본 자세 설정만의 결과"도 아니고
"정책만의 결과"도 아니다.
정확히는 **기본 자세를 기준으로 정책이 작은 보정값을 계속 출력해 안정화하는 구조** 다.

### 4. actuator 정합성

초기 통합 단계에서는 `IsaacLab` 기본 Go2 asset를 사용했지만,
현재 시뮬 환경은 `unitree_rl_lab`의 `UNITREE_GO2_CFG`를 직접 로드하도록 변경했다.

따라서 현재 `go2_sim.py` 경로의 Go2는 다음 항목이 `unitree_rl_lab` 기준과 맞춰진 상태다.

- 기본 자세 (`init_state.joint_pos`)
- Go2 전용 actuator 설정
- `use_default_offset=True`, `scale=0.25` 기반의 관절 목표 해석

이 변경의 목적은
"정책은 unitree_rl_lab에서 학습됐는데 시뮬 모터 모델은 Isaac Lab 기본값"
이라는 불일치를 줄여,
시뮬에서의 관절 응답이 학습/배포 기준과 더 가깝게 동작하게 만드는 것이다.

### 5. 가중치 로드 (`go2_sim.py`)

unitree_rl_lab은 actor(45-dim) / critic(60-dim) obs를 분리 학습함.
현재 env에 critic obs group이 없으므로 → critic 가중치 제외 후 로드:

```python
_ckpt = torch.load(resume_path, weights_only=False)
_actor_state = {k: v for k, v in _ckpt["model_state_dict"].items()
                if not k.startswith("critic")}
runner.alg.policy.load_state_dict(_actor_state, strict=False)
```

> **왜 `strict=False`만으론 안 되나?**
> PyTorch는 키가 존재하는데 크기가 다르면 strict 여부와 관계없이 에러를 냄.
> critic 키 자체를 dict에서 제거해야 "없는 키 → 무시(strict=False)"로 처리됨.

> **critic obs 60-dim 구성** (추론 시 미사용):
> 기본 45-dim + `base_lin_vel`(3) + `joint_effort`(12) = 60-dim

---

## 관련 문서

- [[03_nav2_plan]] — Nav2 자율주행 구현 계획
