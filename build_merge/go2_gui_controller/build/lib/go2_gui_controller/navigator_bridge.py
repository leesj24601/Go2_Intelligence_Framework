from __future__ import annotations

from math import cos, sin

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped

from .state_bridge import StateBridge
from .waypoint_registry import WaypointRegistry


class NavigatorBridge:
    def __init__(self, node, state_bridge: StateBridge, waypoint_registry: WaypointRegistry):
        self._node = node
        self._state_bridge = state_bridge
        self._waypoint_registry = waypoint_registry
        self._goal_pose_publisher = node.create_publisher(PoseStamped, "/goal_pose", 10)
        self._cancel_client = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self._current_goal_label = None

    def start_wait_until_active(self) -> None:
        self._state_bridge.set_nav_status("ready")

    def _refresh_ready_state(self) -> None:
        if not str(self._state_bridge.state.nav_status).startswith("navigating:"):
            self._state_bridge.set_nav_status("ready")

    def _publish_goal_pose(self, pose: PoseStamped, label: str) -> None:
        self._refresh_ready_state()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        self._goal_pose_publisher.publish(pose)
        self._current_goal_label = label
        self._state_bridge.set_nav_status(f"navigating:{label}")

    def go_to_waypoint(self, waypoint_name: str) -> None:
        waypoint = self._waypoint_registry.get(waypoint_name)
        if waypoint is None:
            raise ValueError(f"Unknown waypoint: {waypoint_name}")
        self._publish_goal_pose(waypoint.to_pose_stamped(), waypoint.name)

    def go_to_relative_pose(self, x_m: float, y_m: float, yaw_deg: float = 0.0) -> None:
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
        self._refresh_ready_state()
        if not self._cancel_client.wait_for_service(timeout_sec=0.2):
            self._state_bridge.set_nav_status("waiting_for_nav2")
            return
        request = CancelGoal.Request()
        self._cancel_client.call_async(request)
        self._state_bridge.set_nav_status("cancel_requested")

    def spin_once(self) -> None:
        self._refresh_ready_state()
