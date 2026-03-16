from __future__ import annotations

from dataclasses import dataclass
from math import atan2
from math import cos, sin
from math import hypot

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSPresetProfiles,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0
    frame_id: str = "odom"
    localization_initialized: bool = False
    map_odom_seen: bool = False
    map_base_link_seen: bool = False
    map_camera_link_seen: bool = False
    nav_status: str = "idle"
    last_result: str = "none"


class StateBridge:
    CAMERA_OFFSET_X_M = 0.30
    LOCALIZATION_CHILD_FRAMES = {"odom", "base_link", "camera_link", "base_footprint"}
    MAP_ODOM_TRANSLATION_THRESHOLD_M = 0.03
    MAP_ODOM_YAW_THRESHOLD_RAD = 0.05

    def __init__(self, node, odom_topic: str = "/odom"):
        self._node = node
        self._odom_topic = odom_topic
        self.state = RobotState()
        self._tf_buffer = Buffer()
        self._latest_map_odom_pose: tuple[float, float, float] | None = None
        self._localization_baseline_pose: tuple[float, float, float] | None = None
        self._latest_map_pose: tuple[float, float, float] | None = None
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
        )
        self._subscription = node.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            sensor_qos,
        )
        self._tf_subscription = node.create_subscription(
            TFMessage,
            "/tf",
            self._on_tf_message,
            tf_qos,
        )
        self._tf_static_subscription = node.create_subscription(
            TFMessage,
            "/tf_static",
            self._on_tf_message,
            tf_static_qos,
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.frame_id = self._normalize_frame(msg.header.frame_id) or "odom"
        orientation = msg.pose.pose.orientation
        self.state.yaw_rad = self._yaw_from_quaternion(orientation)

    def update_from_tf(self) -> None:
        try:
            map_odom_transform = self._tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        else:
            self._update_map_odom_tracking(map_odom_transform)

        frame_pairs = (
            ("map", "base_link"),
            ("map", "camera_link"),
            ("odom", "base_link"),
            ("odom", "camera_link"),
        )
        for target_frame, source_frame in frame_pairs:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            self._apply_transform(target_frame, source_frame, transform)
            if target_frame == "map":
                self._mark_frame_seen(source_frame)
            return

    def _on_tf_message(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            parent_frame = self._normalize_frame(transform.header.frame_id)
            child_frame = self._normalize_frame(transform.child_frame_id)
            if parent_frame == "map" and child_frame in self.LOCALIZATION_CHILD_FRAMES:
                self._mark_frame_seen(child_frame)

    def reset_localization_tracking(self) -> None:
        self.state.localization_initialized = False
        self.state.map_odom_seen = False
        self.state.map_base_link_seen = False
        self.state.map_camera_link_seen = False
        self._localization_baseline_pose = self._latest_map_odom_pose
        self._latest_map_pose = (self.state.x, self.state.y, self.state.yaw_rad) if self.state.frame_id == "map" else None

    def _mark_frame_seen(self, child_frame: str) -> None:
        normalized_child = self._normalize_frame(child_frame)
        if normalized_child == "odom":
            self.state.map_odom_seen = True
        elif normalized_child == "base_link":
            self.state.map_base_link_seen = True
        elif normalized_child == "camera_link":
            self.state.map_camera_link_seen = True

    def _update_map_odom_tracking(self, transform) -> None:
        current_pose = self._extract_planar_pose(transform.transform.translation, transform.transform.rotation)
        self._latest_map_odom_pose = current_pose
        self._mark_frame_seen("odom")

        if self.state.localization_initialized:
            return

        baseline_pose = self._localization_baseline_pose
        if baseline_pose is None:
            self._localization_baseline_pose = current_pose
            return

        if self._pose_delta_is_significant(baseline_pose, current_pose):
            self.state.localization_initialized = True

    def update_localization_from_map_pose(self, x_value: float, y_value: float, yaw_rad: float) -> None:
        if self.state.localization_initialized:
            return
        current_pose = (x_value, y_value, yaw_rad)
        baseline_pose = self._latest_map_pose
        if baseline_pose is None:
            self._latest_map_pose = current_pose
            return
        if self._pose_delta_is_significant(baseline_pose, current_pose):
            self.state.localization_initialized = True
            self._latest_map_pose = current_pose
            return
        self._latest_map_pose = current_pose

    def _extract_planar_pose(self, translation, rotation) -> tuple[float, float, float]:
        yaw_rad = self._yaw_from_quaternion(rotation)
        return (float(translation.x), float(translation.y), yaw_rad)

    def _pose_delta_is_significant(
        self,
        baseline_pose: tuple[float, float, float],
        current_pose: tuple[float, float, float],
    ) -> bool:
        baseline_x, baseline_y, baseline_yaw = baseline_pose
        current_x, current_y, current_yaw = current_pose
        translation_delta = hypot(current_x - baseline_x, current_y - baseline_y)
        yaw_delta = abs(self._normalize_angle(current_yaw - baseline_yaw))
        return (
            translation_delta >= self.MAP_ODOM_TRANSLATION_THRESHOLD_M
            or yaw_delta >= self.MAP_ODOM_YAW_THRESHOLD_RAD
        )

    def _yaw_from_quaternion(self, rotation) -> float:
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle_rad: float) -> float:
        while angle_rad > 3.141592653589793:
            angle_rad -= 2.0 * 3.141592653589793
        while angle_rad < -3.141592653589793:
            angle_rad += 2.0 * 3.141592653589793
        return angle_rad

    def _apply_transform(self, target_frame: str, source_frame: str, transform) -> None:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw_rad = self._yaw_from_quaternion(rotation)

        x_value = translation.x
        y_value = translation.y
        if source_frame == "camera_link":
            # RViz can localize using map->camera_link even when base_link is not directly resolvable.
            x_value -= self.CAMERA_OFFSET_X_M * cos(yaw_rad)
            y_value -= self.CAMERA_OFFSET_X_M * sin(yaw_rad)

        self.state.x = x_value
        self.state.y = y_value
        self.state.frame_id = self._normalize_frame(target_frame)
        self.state.yaw_rad = yaw_rad
        if target_frame == "map":
            self.update_localization_from_map_pose(x_value, y_value, yaw_rad)

    def _normalize_frame(self, frame_id: str) -> str:
        return str(frame_id or "").lstrip("/")

    def set_nav_status(self, status: str) -> None:
        self.state.nav_status = status

    def set_last_result(self, result: str) -> None:
        self.state.last_result = result

    def pose_text(self) -> str:
        yaw_deg = self.state.yaw_rad * 180.0 / 3.141592653589793
        return f"{self.state.frame_id}: x={self.state.x:.2f}, y={self.state.y:.2f}, yaw={yaw_deg:.1f}deg"
