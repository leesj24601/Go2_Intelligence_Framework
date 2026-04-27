"""Isaac Lab Go2 SLAM simulation environment."""

import math
import os
from pathlib import Path

from isaaclab.utils import configclass
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors import CameraCfg, ImuCfg
import isaaclab.sim as sim_utils
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import (
    UnitreeGo2RoughEnvCfg,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_USD_PATH = PROJECT_ROOT / "assets" / "office.usd"


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return float(value)


def _yaw_to_quat_wxyz(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw = yaw_rad * 0.5
    return (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))


@configclass
class MySlamEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        usd_path = os.environ.get("GO2_SLAM_USD_PATH", str(DEFAULT_USD_PATH))
        spawn_x = _env_float("GO2_SPAWN_X", 0.5)
        spawn_y = _env_float("GO2_SPAWN_Y", 0.0)
        spawn_z = _env_float("GO2_SPAWN_Z", 0.4)
        spawn_yaw = _env_float("GO2_SPAWN_YAW", math.pi)

        # 1. SLAM/semantic-map 테스트용 USD 경로 지정
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="usd",
            usd_path=usd_path,
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

        # Office USD 기준: 리셉션 카운터 전면 중앙에 Go2를 고정 스폰한다.
        # 필요하면 GO2_SPAWN_X/Y/Z/YAW 환경변수로 로컬 조정한다.
        self.scene.robot.init_state.pos = (spawn_x, spawn_y, spawn_z)
        self.scene.robot.init_state.rot = _yaw_to_quat_wxyz(spawn_yaw)
        self.events.reset_base.params = {
            "pose_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

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
            width=320,
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
