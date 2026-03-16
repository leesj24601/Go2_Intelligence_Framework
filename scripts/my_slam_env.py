# my_slam_env.py
import sys
from pathlib import Path

from isaaclab.utils import configclass
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors import CameraCfg, ImuCfg
import isaaclab.sim as sim_utils
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import UnitreeGo2RoughEnvCfg


def _load_unitree_rl_lab_go2_cfg():
    """Load the Go2 asset config from unitree_rl_lab so actuator behavior matches policy training."""
    repo_root = Path(__file__).resolve().parents[1]
    unitree_rl_lab_src = repo_root.parent / "unitree_rl_lab" / "source" / "unitree_rl_lab"
    if not unitree_rl_lab_src.exists():
        raise FileNotFoundError(f"unitree_rl_lab source path not found: {unitree_rl_lab_src}")

    unitree_rl_lab_src_str = str(unitree_rl_lab_src)
    if unitree_rl_lab_src_str not in sys.path:
        sys.path.insert(0, unitree_rl_lab_src_str)

    from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG

    return UNITREE_GO2_CFG


UNITREE_RL_LAB_GO2_CFG = _load_unitree_rl_lab_go2_cfg()


@configclass
class MySlamEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = UNITREE_RL_LAB_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # 1. 새로 저장한 SLAM 전용 USD 경로 지정
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="usd",
            usd_path="/home/cvr/Desktop/sj/go2_intelligence_framework/assets/slam_env.usd",
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
        )

        # 2. 센서가 창고 바닥과 그 하위 기둥들을 모두 인식하도록 설정
        if hasattr(self.scene, "height_scanner"):
            # 바구니 통합이 완료되면 이 주소 하나면 충분합니다.
            self.scene.height_scanner.mesh_prim_paths = ["/World/ground"]
            self.scene.height_scanner.debug_vis = False

        # 3. 제어 설정 유지
        if hasattr(self.commands, "base_velocity"):
            self.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
            self.commands.base_velocity.debug_vis = False
            # [중요] Heading command를 꺼야 사용자의 Q/E 회전 명령이 직접 전달됩니다.
            self.commands.base_velocity.heading_command = False

        self.episode_length_s = 1.0e9
        if hasattr(self.curriculum, "terrain_levels"):
            self.curriculum.terrain_levels = None

        # Unitree RL Lab 정책 obs space 맞추기 (45-dim)
        # 제거: base_lin_vel(3), height_scan(~187)
        # 유지: base_ang_vel(3×0.2), projected_gravity(3), velocity_commands(3),
        #        joint_pos_rel(12), joint_vel_rel(12×0.05), last_action(12)
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None
        self.observations.policy.base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel, scale=0.2, noise=Unoise(n_min=-0.2, n_max=0.2)
        )
        self.observations.policy.joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, scale=0.05, noise=Unoise(n_min=-1.5, n_max=1.5)
        )

        # IMU 센서 (50Hz, body frame)
        self.scene.imu_sensor = ImuCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base",
            update_period=1.0 / 50.0,
            gravity_bias=(0.0, 0.0, 9.81),
        )

        # Intel RealSense D435 근사 카메라
        self.scene.front_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
            update_period=0,  # 센서 데이터 수집 비활성화 (ROS2는 숨겨진 뷰포트 사용)
            height=240,
            width=424,
            data_types=[],  # prim만 생성, 센서 렌더링 안 함 (이중 렌더링 방지)
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=15.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 50.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.30, 0.0, 0.05),
                rot=(0.5, -0.5, 0.5, -0.5),
                convention="ros",
            ),
        )
