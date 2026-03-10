from __future__ import annotations

from math import cos, sin
from threading import Lock, Thread
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .state_bridge import StateBridge
from .waypoint_registry import WaypointRegistry


class NavigatorBridge:
    def __init__(self, state_bridge: StateBridge, waypoint_registry: WaypointRegistry):
        self._navigator = BasicNavigator()
        self._state_bridge = state_bridge
        self._waypoint_registry = waypoint_registry
        self._current_goal_label: Optional[str] = None
        self._nav_ready = False
        self._nav_wait_started = False
        self._nav_wait_lock = Lock()

    def wait_until_active(self) -> None:
        self._state_bridge.set_nav_status("waiting_for_nav2")
        self._navigator.waitUntilNav2Active()
        self._state_bridge.set_nav_status("ready")
        self._nav_ready = True

    def start_wait_until_active(self) -> None:
        with self._nav_wait_lock:
            if self._nav_ready or self._nav_wait_started:
                return
            self._nav_wait_started = True
        self._state_bridge.set_nav_status("waiting_for_nav2")
        Thread(target=self._wait_until_active_worker, daemon=True).start()

    def _wait_until_active_worker(self) -> None:
        try:
            self._navigator.waitUntilNav2Active()
            self._state_bridge.set_nav_status("ready")
            self._nav_ready = True
        except Exception as exc:
            self._state_bridge.set_last_result(f"nav2_wait_failed:{exc}")
            self._state_bridge.set_nav_status("nav2_wait_failed")

    def ensure_active(self) -> None:
        if self._nav_ready:
            return
        self.start_wait_until_active()
        raise RuntimeError("Nav2 is not active yet")

    def go_to_waypoint(self, waypoint_name: str) -> None:
        self.ensure_active()
        waypoint = self._waypoint_registry.get(waypoint_name)
        if waypoint is None:
            raise ValueError(f"Unknown waypoint: {waypoint_name}")
        self._current_goal_label = waypoint.name
        self._navigator.goToPose(waypoint.to_pose_stamped())
        self._state_bridge.set_nav_status(f"navigating:{waypoint.name}")

    def go_to_relative_pose(self, x_m: float, y_m: float, yaw_deg: float = 0.0) -> None:
        self.ensure_active()
        current_x = self._state_bridge.state.x
        current_y = self._state_bridge.state.y
        current_yaw = self._state_bridge.state.yaw_rad
        map_dx = x_m * cos(current_yaw) - y_m * sin(current_yaw)
        map_dy = x_m * sin(current_yaw) + y_m * cos(current_yaw)
        target_yaw = current_yaw + (yaw_deg * 3.141592653589793 / 180.0)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = current_x + map_dx
        pose.pose.position.y = current_y + map_dy
        half_yaw = target_yaw / 2.0
        pose.pose.orientation.z = sin(half_yaw)
        pose.pose.orientation.w = cos(half_yaw)
        self._current_goal_label = "relative_goal"
        self._navigator.goToPose(pose)
        self._state_bridge.set_nav_status("navigating:relative_goal")

    def cancel(self) -> None:
        if not self._nav_ready:
            self._state_bridge.set_nav_status("waiting_for_nav2")
            return
        self._navigator.cancelTask()
        self._state_bridge.set_nav_status("cancel_requested")

    def spin_once(self) -> None:
        if not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            if feedback is not None and self._current_goal_label:
                self._state_bridge.set_nav_status(f"navigating:{self._current_goal_label}")
            return

        result = self._navigator.getResult()
        if result is None:
            return

        if result == TaskResult.SUCCEEDED:
            result_text = "succeeded"
        elif result == TaskResult.CANCELED:
            result_text = "canceled"
        elif result == TaskResult.FAILED:
            result_text = "failed"
        else:
            result_text = "unknown"

        self._state_bridge.set_last_result(result_text)
        self._state_bridge.set_nav_status("idle")
        self._current_goal_label = None
