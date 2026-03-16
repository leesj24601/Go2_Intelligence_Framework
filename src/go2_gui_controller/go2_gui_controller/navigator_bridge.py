from __future__ import annotations

from math import cos, sin

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from .state_bridge import StateBridge
from .waypoint_registry import WaypointRegistry


class NavigatorBridge:
    def __init__(self, node, state_bridge: StateBridge, waypoint_registry: WaypointRegistry):
        self._node = node
        self._state_bridge = state_bridge
        self._waypoint_registry = waypoint_registry
        self._navigate_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        self._current_goal_handle = None
        self._current_goal_label: str | None = None
        self._pending_goal_label: str | None = None

    def start_wait_until_active(self) -> None:
        if not self.server_ready():
            self._state_bridge.set_nav_status("waiting_for_nav2")
            return
        if not self.localization_ready():
            self._state_bridge.set_nav_status("waiting_for_localization")
            return
        self._state_bridge.set_nav_status("ready")

    def _publish_goal_pose(self, pose: PoseStamped, label: str) -> None:
        if not self.server_ready():
            raise RuntimeError("Nav2 action server unavailable")
        if not self.localization_ready():
            raise RuntimeError("Localization unavailable (map->odom not seen)")

        pose.header.stamp = self._node.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._pending_goal_label = label
        self._state_bridge.set_nav_status(f"sending_goal:{label}")
        send_future = self._navigate_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def go_to_waypoint(self, waypoint_name: str) -> None:
        waypoint = self._waypoint_registry.get(waypoint_name)
        if waypoint is None:
            raise ValueError(f"Unknown waypoint: {waypoint_name}")
        self._publish_goal_pose(waypoint.to_pose_stamped(), waypoint.name)

    def go_to_relative_pose(self, x_m: float, y_m: float, yaw_deg: float = 0.0) -> None:
        if self._state_bridge.state.frame_id != "map":
            raise RuntimeError("map pose unavailable")
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
        self._publish_goal_pose(pose, "relative_goal")

    def cancel(self) -> None:
        if not self.server_ready():
            self._state_bridge.set_nav_status("waiting_for_nav2")
            return

        if self._current_goal_handle is None:
            if self.localization_ready():
                self._state_bridge.set_nav_status("ready")
            else:
                self._state_bridge.set_nav_status("waiting_for_localization")
            return

        self._current_goal_handle.cancel_goal_async()
        self._state_bridge.set_nav_status("cancel_requested")

    def spin_once(self) -> None:
        if self._current_goal_handle is not None or self._pending_goal_label is not None:
            return
        if self.server_ready():
            if self.localization_ready():
                if self._state_bridge.state.nav_status in ("idle", "waiting_for_nav2", "waiting_for_localization"):
                    self._state_bridge.set_nav_status("ready")
                return
            if self._state_bridge.state.nav_status in ("idle", "ready", "waiting_for_nav2", "waiting_for_localization"):
                self._state_bridge.set_nav_status("waiting_for_localization")
            return
        if self._state_bridge.state.nav_status in ("idle", "ready", "waiting_for_nav2", "waiting_for_localization"):
            self._state_bridge.set_nav_status("waiting_for_nav2")

    def server_ready(self) -> bool:
        return self._navigate_client.server_is_ready()

    def localization_ready(self) -> bool:
        return self._state_bridge.state.localization_initialized

    def navigation_ready(self) -> bool:
        return self.server_ready() and self.localization_ready()

    def _on_goal_response(self, future) -> None:
        label = self._pending_goal_label or "goal"
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._pending_goal_label = None
            self._current_goal_handle = None
            self._current_goal_label = None
            self._state_bridge.set_nav_status("goal_send_failed")
            self._state_bridge.set_last_result(f"send_failed:{label} ({exc})")
            return

        self._pending_goal_label = None
        if not goal_handle.accepted:
            self._current_goal_handle = None
            self._current_goal_label = None
            self._state_bridge.set_nav_status("goal_rejected")
            self._state_bridge.set_last_result(f"rejected:{label}")
            return

        self._current_goal_handle = goal_handle
        self._current_goal_label = label
        self._state_bridge.set_nav_status(f"navigating:{label}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_feedback(self, feedback_msg) -> None:
        label = self._current_goal_label or self._pending_goal_label or "goal"
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            self._state_bridge.set_nav_status(f"navigating:{label} ({distance_remaining:.2f}m)")
            return
        self._state_bridge.set_nav_status(f"navigating:{label}")

    def _on_goal_result(self, future) -> None:
        label = self._current_goal_label or "goal"
        try:
            result = future.result()
        except Exception as exc:
            self._state_bridge.set_nav_status("goal_result_failed")
            self._state_bridge.set_last_result(f"result_failed:{label} ({exc})")
            self._clear_goal_tracking()
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._state_bridge.set_nav_status("succeeded")
            self._state_bridge.set_last_result(f"succeeded:{label}")
        elif status == GoalStatus.STATUS_CANCELED:
            self._state_bridge.set_nav_status("canceled")
            self._state_bridge.set_last_result(f"canceled:{label}")
        elif status == GoalStatus.STATUS_ABORTED:
            self._state_bridge.set_nav_status("failed")
            self._state_bridge.set_last_result(f"aborted:{label}")
        else:
            self._state_bridge.set_nav_status("failed")
            self._state_bridge.set_last_result(f"status_{status}:{label}")
        self._clear_goal_tracking()

    def _clear_goal_tracking(self) -> None:
        self._current_goal_handle = None
        self._current_goal_label = None
        self._pending_goal_label = None
