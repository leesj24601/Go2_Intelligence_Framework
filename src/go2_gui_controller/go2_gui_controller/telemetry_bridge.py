from __future__ import annotations

from collections import deque
from math import hypot
import time
from typing import Deque

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    DurabilityPolicy,
    QoSPresetProfiles,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState, LaserScan


class TelemetryBridge:
    def __init__(self, node, odom_topic: str = "/odom", history_sec: float = 20.0, max_samples: int = 1200):
        self._node = node
        self._odom_topic = odom_topic
        self._history_sec = history_sec
        self._max_samples = max_samples

        self._joint_position_buffers: dict[str, Deque[tuple[float, float]]] = {}
        self._joint_velocity_buffers: dict[str, Deque[tuple[float, float]]] = {}
        self._cmd_linear_buffer: Deque[tuple[float, float]] = deque(maxlen=max_samples)
        self._cmd_angular_buffer: Deque[tuple[float, float]] = deque(maxlen=max_samples)
        self._odom_linear_buffer: Deque[tuple[float, float]] = deque(maxlen=max_samples)
        self._odom_angular_buffer: Deque[tuple[float, float]] = deque(maxlen=max_samples)
        self._topic_status = {
            "/joint_states": {"last": 0.0, "count": 0},
            odom_topic: {"last": 0.0, "count": 0},
            "/scan": {"last": 0.0, "count": 0},
            "/map": {"last": 0.0, "count": 0},
            "/cmd_vel": {"last": 0.0, "count": 0},
        }

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._joint_state_subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self._odom_subscription = node.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            sensor_qos,
        )
        self._cmd_vel_subscription = node.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            10,
        )
        self._scan_subscription = node.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            sensor_qos,
        )
        self._map_subscription = node.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            map_qos,
        )

    @property
    def history_sec(self) -> float:
        return self._history_sec

    def joint_names(self) -> list[str]:
        return sorted(set(self._joint_position_buffers) | set(self._joint_velocity_buffers))

    def get_joint_position_series(self, joint_name: str, now_sec: float | None = None) -> tuple[list[float], list[float]]:
        return self._buffer_to_plot(self._joint_position_buffers.get(joint_name), now_sec)

    def get_joint_velocity_series(self, joint_name: str, now_sec: float | None = None) -> tuple[list[float], list[float]]:
        return self._buffer_to_plot(self._joint_velocity_buffers.get(joint_name), now_sec)

    def get_cmd_linear_series(self, now_sec: float | None = None) -> tuple[list[float], list[float]]:
        return self._buffer_to_plot(self._cmd_linear_buffer, now_sec)

    def get_cmd_angular_series(self, now_sec: float | None = None) -> tuple[list[float], list[float]]:
        return self._buffer_to_plot(self._cmd_angular_buffer, now_sec)

    def get_odom_linear_series(self, now_sec: float | None = None) -> tuple[list[float], list[float]]:
        return self._buffer_to_plot(self._odom_linear_buffer, now_sec)

    def get_odom_angular_series(self, now_sec: float | None = None) -> tuple[list[float], list[float]]:
        return self._buffer_to_plot(self._odom_angular_buffer, now_sec)

    def topic_summary_lines(self) -> list[str]:
        now_sec = time.monotonic()
        lines = []
        for topic_name in ("/joint_states", self._odom_topic, "/cmd_vel"):
            status = self._topic_status[topic_name]
            if status["count"] <= 0:
                lines.append(f"{topic_name}: no data")
                continue
            age_sec = max(now_sec - status["last"], 0.0)
            lines.append(f"{topic_name}: {status['count']} msgs, last {age_sec:.1f}s ago")
        return lines

    def has_topic_data(self, topic_name: str) -> bool:
        status = self._topic_status.get(topic_name)
        return bool(status and status["count"] > 0)

    def has_recent_topic_data(self, topic_name: str, max_age_sec: float) -> bool:
        status = self._topic_status.get(topic_name)
        if not status or status["count"] <= 0:
            return False
        return (time.monotonic() - status["last"]) <= max_age_sec

    def topic_age_sec(self, topic_name: str) -> float | None:
        status = self._topic_status.get(topic_name)
        if not status or status["count"] <= 0:
            return None
        return max(time.monotonic() - status["last"], 0.0)

    def _on_joint_state(self, msg: JointState) -> None:
        now_sec = time.monotonic()
        self._mark_topic("/joint_states", now_sec)
        for index, joint_name in enumerate(msg.name):
            if index < len(msg.position):
                self._append_value(
                    self._joint_position_buffers.setdefault(joint_name, deque(maxlen=self._max_samples)),
                    float(msg.position[index]),
                    now_sec,
                )
            if index < len(msg.velocity):
                self._append_value(
                    self._joint_velocity_buffers.setdefault(joint_name, deque(maxlen=self._max_samples)),
                    float(msg.velocity[index]),
                    now_sec,
                )

    def _on_odom(self, msg: Odometry) -> None:
        now_sec = time.monotonic()
        self._mark_topic(self._odom_topic, now_sec)
        twist = msg.twist.twist
        linear_speed = hypot(twist.linear.x, twist.linear.y)
        self._append_value(self._odom_linear_buffer, linear_speed, now_sec)
        self._append_value(self._odom_angular_buffer, float(twist.angular.z), now_sec)

    def _on_cmd_vel(self, msg: Twist) -> None:
        now_sec = time.monotonic()
        self._mark_topic("/cmd_vel", now_sec)
        linear_speed = hypot(msg.linear.x, msg.linear.y)
        self._append_value(self._cmd_linear_buffer, linear_speed, now_sec)
        self._append_value(self._cmd_angular_buffer, float(msg.angular.z), now_sec)

    def _on_scan(self, _msg: LaserScan) -> None:
        self._mark_topic("/scan", time.monotonic())

    def _on_map(self, _msg: OccupancyGrid) -> None:
        self._mark_topic("/map", time.monotonic())

    def _mark_topic(self, topic_name: str, now_sec: float) -> None:
        status = self._topic_status[topic_name]
        status["last"] = now_sec
        status["count"] += 1

    def _append_value(self, buffer: Deque[tuple[float, float]], value: float, now_sec: float) -> None:
        buffer.append((now_sec, value))
        cutoff_sec = now_sec - self._history_sec
        while buffer and buffer[0][0] < cutoff_sec:
            buffer.popleft()

    def _buffer_to_plot(
        self,
        buffer: Deque[tuple[float, float]] | None,
        now_sec: float | None,
    ) -> tuple[list[float], list[float]]:
        if not buffer:
            return [], []
        sample_now_sec = now_sec if now_sec is not None else time.monotonic()
        x_values = [timestamp - sample_now_sec for timestamp, _ in buffer]
        y_values = [value for _, value in buffer]
        return x_values, y_values
