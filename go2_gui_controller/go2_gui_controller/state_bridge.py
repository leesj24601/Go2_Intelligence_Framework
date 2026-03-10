from __future__ import annotations

from dataclasses import dataclass
from math import atan2

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0
    frame_id: str = "odom"
    nav_status: str = "idle"
    last_result: str = "none"


class StateBridge:
    def __init__(self, node):
        self._node = node
        self.state = RobotState()
        self._tf_buffer = Buffer()
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self._tf_listener = TransformListener(
            self._tf_buffer,
            node,
            spin_thread=False,
            qos=sensor_qos,
            static_qos=sensor_qos,
        )
        self._subscription = node.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            sensor_qos,
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        self.state.frame_id = msg.header.frame_id or "odom"
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.state.yaw_rad = atan2(siny_cosp, cosy_cosp)

    def update_from_tf(self) -> None:
        for target_frame, source_frame in (("map", "base_link"), ("odom", "base_link")):
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.state.x = translation.x
            self.state.y = translation.y
            self.state.frame_id = target_frame
            siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
            cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
            self.state.yaw_rad = atan2(siny_cosp, cosy_cosp)
            return

    def set_nav_status(self, status: str) -> None:
        self.state.nav_status = status

    def set_last_result(self, result: str) -> None:
        self.state.last_result = result

    def pose_text(self) -> str:
        yaw_deg = self.state.yaw_rad * 180.0 / 3.141592653589793
        return f"{self.state.frame_id}: x={self.state.x:.2f}, y={self.state.y:.2f}, yaw={yaw_deg:.1f}deg"
