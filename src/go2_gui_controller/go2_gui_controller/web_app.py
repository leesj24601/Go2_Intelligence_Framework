from __future__ import annotations

import asyncio
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from starlette.concurrency import run_in_threadpool
import uvicorn

from .commands import CommandType, ParsedCommand
from .manual_control import ManualControlBridge
from .navigator_bridge import NavigatorBridge
from .state_bridge import StateBridge
from .telemetry_bridge import TelemetryBridge
from .text_command_parser import TextCommandParser
from .voice_command_listener import VoiceCommandListener
from .waypoint_registry import WaypointRegistry
from .web_launch_manager import WebLaunchManager


RUNTIME_MODES = {
    "sim": {"label": "Simulation", "odom_topic": "/odom", "use_sim_time": True},
    "real": {"label": "Real Robot", "odom_topic": "/utlidar/robot_odom", "use_sim_time": False},
}

MANUAL_SPEEDS = {
    "forward": (1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "left": (0.0, 0.6, 0.0),
    "right": (0.0, -0.6, 0.0),
    "turn_l": (0.0, 0.0, 1.0),
    "turn_r": (0.0, 0.0, -1.0),
}


class ManualCommandRequest(BaseModel):
    direction: str


class WaypointNavigateRequest(BaseModel):
    waypoint_name: str


class TextCommandRequest(BaseModel):
    text: str


class SaveWaypointRequest(BaseModel):
    name: str


class DeleteWaypointRequest(BaseModel):
    name: str


class RenameWaypointRequest(BaseModel):
    old_name: str
    new_name: str


class StackTargetRequest(BaseModel):
    target: str


class WebControllerNode(Node):
    VOICE_LINEAR_X = 0.35
    VOICE_LINEAR_Y = 0.25
    VOICE_ANGULAR_Z = 0.8
    VOICE_JOG_SEC = 0.8
    VOICE_FINE_JOG_SEC = 0.4

    def __init__(self, default_waypoint_file: Path):
        super().__init__("go2_web_controller")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("waypoint_file", "")
        self._declare_or_set_parameter("host", "127.0.0.1")
        self._declare_or_set_parameter("port", 8080)

        mode_key = str(self.get_parameter("mode").value or "sim").lower()
        self.runtime_mode_key = mode_key if mode_key in RUNTIME_MODES else "sim"
        self.runtime_mode = RUNTIME_MODES[self.runtime_mode_key]
        self.set_parameters([Parameter("use_sim_time", value=self.runtime_mode["use_sim_time"])])

        waypoint_file_param = str(self.get_parameter("waypoint_file").value or "").strip()
        waypoint_file = Path(waypoint_file_param) if waypoint_file_param else default_waypoint_file

        self._lock = threading.Lock()
        self._logs: list[str] = []
        self._pending_log: str | None = None
        self._stack_status = {"slam": "stopped", "navigation": "stopped", "rviz": "stopped"}
        self._running = True

        self.waypoint_registry = WaypointRegistry(waypoint_file)
        self.state_bridge = StateBridge(self, odom_topic=self.runtime_mode["odom_topic"])
        self.telemetry_bridge = TelemetryBridge(self, odom_topic=self.runtime_mode["odom_topic"])
        self.navigator_bridge = NavigatorBridge(self, self.state_bridge, self.waypoint_registry)
        self.manual_bridge = ManualControlBridge(self)
        self.text_parser = TextCommandParser(self.waypoint_registry)
        self.voice_listener = VoiceCommandListener()
        self.launch_manager = WebLaunchManager(
            runtime_mode_key=self.runtime_mode_key,
            log_callback=self._append_log,
            status_callback=self._status_callback,
        )
        if not self.launch_manager.available:
            self._append_log(f"[Runtime] launch manager unavailable ({self.launch_manager.unavailable_reason})")

    def _declare_or_set_parameter(self, name: str, value) -> None:
        if self.has_parameter(name):
            self.set_parameters([Parameter(name, value=value)])
            return
        self.declare_parameter(name, value)

    def _status_callback(self, key: str, status: str) -> None:
        with self._lock:
            self._stack_status[key] = status

    def _append_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"{timestamp} {message}"
        with self._lock:
            self._logs.append(line)
            if len(self._logs) > 200:
                self._logs.pop(0)
            self._pending_log = line

    def get_logs(self) -> list[str]:
        with self._lock:
            return list(self._logs)

    def clear_logs(self) -> None:
        with self._lock:
            self._logs.clear()
            self._pending_log = None

    def get_state_snapshot(self) -> dict:
        with self._lock:
            state = self.state_bridge.state
            return {
                "mode": self.runtime_mode_key,
                "mode_label": self.runtime_mode["label"],
                "pose": {
                    "x": state.x,
                    "y": state.y,
                    "yaw_deg": math.degrees(state.yaw_rad),
                    "frame_id": state.frame_id,
                },
                "nav_status": state.nav_status,
                "last_result": state.last_result,
                "localized": state.localization_initialized,
                "waypoints": [w.name for w in self.waypoint_registry.list_waypoints()],
                "stacks": dict(self._stack_status),
                "voice_available": self.voice_listener.available,
                "voice_status": "ready" if self.voice_listener.available else self.voice_listener.unavailable_reason,
                "log": self._pending_log,
            }

    def clear_pending_log(self) -> None:
        with self._lock:
            self._pending_log = None

    def stop_runtime(self) -> None:
        self._running = False

    def is_running(self) -> bool:
        return self._running


def _voice_prefers_manual(command: ParsedCommand) -> bool:
    if command.command_type not in (CommandType.MOVE_RELATIVE, CommandType.ROTATE_RELATIVE):
        return False
    source_text = command.source_text.lower()
    if any(character.isdigit() for character in source_text):
        return False
    return not any(
        token in source_text
        for token in ("미터", "meter", "meters", "cm", "도", "deg", "degree", "degrees", "반바퀴")
    )


def _publish_for_duration(
    node: WebControllerNode,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    duration_sec: float,
    interval_sec: float = 0.05,
) -> None:
    deadline = time.monotonic() + max(duration_sec, interval_sec)
    while time.monotonic() < deadline:
        node.manual_bridge.send_velocity(linear_x, linear_y, angular_z)
        time.sleep(interval_sec)
    node.manual_bridge.stop()


def _execute_voice_manual_command(node: WebControllerNode, command: ParsedCommand) -> str | None:
    if command.command_type == CommandType.STOP:
        node.navigator_bridge.cancel()
        node.manual_bridge.stop()
        node._append_log("[Control] voice stop sent")
        return "stop sent"
    if command.command_type == CommandType.CANCEL_NAVIGATION:
        node.navigator_bridge.cancel()
        node._append_log("[Control] voice navigation cancel requested")
        return "navigation cancelled"
    if not _voice_prefers_manual(command):
        return None

    source_text = command.source_text.lower()
    jog_sec = (
        node.VOICE_FINE_JOG_SEC
        if any(token in source_text for token in ("조금", "살짝"))
        else node.VOICE_JOG_SEC
    )

    if command.command_type == CommandType.MOVE_RELATIVE:
        linear_x = 0.0
        linear_y = 0.0
        if command.x_m > 0.0:
            linear_x = node.VOICE_LINEAR_X
        elif command.x_m < 0.0:
            linear_x = -node.VOICE_LINEAR_X
        elif command.y_m > 0.0:
            linear_y = node.VOICE_LINEAR_Y
        elif command.y_m < 0.0:
            linear_y = -node.VOICE_LINEAR_Y
        node.navigator_bridge.cancel()
        _publish_for_duration(node, linear_x, linear_y, 0.0, jog_sec)
        node._append_log(
            f"[Control] voice cmd_vel x={linear_x:.2f}, y={linear_y:.2f}, yaw=0.00, duration={jog_sec:.2f}s"
        )
        return "voice manual move executed"

    if command.command_type == CommandType.ROTATE_RELATIVE:
        angular_z = node.VOICE_ANGULAR_Z if command.yaw_deg >= 0.0 else -node.VOICE_ANGULAR_Z
        node.navigator_bridge.cancel()
        _publish_for_duration(node, 0.0, 0.0, angular_z, jog_sec)
        node._append_log(
            f"[Control] voice cmd_vel x=0.00, y=0.00, yaw={angular_z:.2f}, duration={jog_sec:.2f}s"
        )
        return "voice manual rotate executed"

    return None


def execute_parsed_command(
    node: WebControllerNode,
    command: ParsedCommand,
    *,
    source_prefix: str,
    prefer_voice_manual: bool = False,
) -> str:
    if prefer_voice_manual:
        manual_result = _execute_voice_manual_command(node, command)
        if manual_result is not None:
            return manual_result

    if command.command_type == CommandType.STOP:
        node.navigator_bridge.cancel()
        node.manual_bridge.stop()
        node._append_log(f"[Control] {source_prefix} stop sent")
        return "stop sent"

    if command.command_type == CommandType.CANCEL_NAVIGATION:
        node.navigator_bridge.cancel()
        node._append_log(f"[Control] {source_prefix} navigation cancel requested")
        return "navigation cancelled"

    if command.command_type == CommandType.NAVIGATE_TO_WAYPOINT and command.waypoint_name:
        node.navigator_bridge.go_to_waypoint(command.waypoint_name)
        node._append_log(f"[Control] {source_prefix} navigate_to_waypoint: {command.waypoint_name}")
        return f"navigating to {command.waypoint_name}"

    if command.command_type == CommandType.MOVE_RELATIVE:
        node.navigator_bridge.go_to_relative_pose(command.x_m, command.y_m, 0.0)
        node._append_log(
            f"[Control] {source_prefix} move_relative: x={command.x_m:.2f}, y={command.y_m:.2f}"
        )
        return f"moving relative x={command.x_m:.2f}m y={command.y_m:.2f}m"

    if command.command_type == CommandType.ROTATE_RELATIVE:
        node.navigator_bridge.go_to_relative_pose(0.0, 0.0, command.yaw_deg)
        node._append_log(f"[Control] {source_prefix} rotate_relative: {command.yaw_deg:.1f} deg")
        return f"rotating {command.yaw_deg:.1f}deg"

    raise RuntimeError("unknown command")


def _ros_spin_loop(node: WebControllerNode) -> None:
    while rclpy.ok() and node.is_running():
        rclpy.spin_once(node, timeout_sec=0.05)
        node.state_bridge.update_from_tf()
        node.navigator_bridge.spin_once()


def create_app(node: WebControllerNode, index_file: Path) -> FastAPI:
    app = FastAPI(title="Go2 Web Controller")

    @app.get("/")
    async def root() -> FileResponse:
        return FileResponse(index_file)

    @app.get("/logs")
    async def get_logs() -> dict:
        return {"logs": node.get_logs()}

    @app.post("/logs/clear")
    async def clear_logs() -> dict:
        node.clear_logs()
        return {"ok": True}

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(node.get_state_snapshot())
                node.clear_pending_log()
                await asyncio.sleep(0.05)
        except WebSocketDisconnect:
            return

    @app.post("/cmd/manual")
    async def manual_command(request: ManualCommandRequest) -> dict:
        if request.direction not in MANUAL_SPEEDS:
            raise HTTPException(status_code=400, detail=f"unknown direction: {request.direction}")
        linear_x, linear_y, angular_z = MANUAL_SPEEDS[request.direction]
        node.navigator_bridge.cancel()
        node.manual_bridge.send_velocity(linear_x, linear_y, angular_z)
        return {"ok": True}

    @app.post("/cmd/stop")
    async def stop_command() -> dict:
        node.navigator_bridge.cancel()
        node.manual_bridge.stop()
        node._append_log("[Control] stop sent")
        return {"ok": True, "message": "stop sent"}

    @app.post("/cmd/navigate")
    async def navigate_command(request: WaypointNavigateRequest) -> dict:
        waypoint_name = request.waypoint_name.strip()
        if not waypoint_name:
            raise HTTPException(status_code=400, detail="waypoint_name is required")
        try:
            node.navigator_bridge.go_to_waypoint(waypoint_name)
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        waypoint = node.waypoint_registry.get(waypoint_name)
        if waypoint is not None:
            node._append_log(
                f"[Control] go_to_waypoint: {waypoint.name} frame={waypoint.frame_id} "
                f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw_deg:.1f}"
            )
        return {"ok": True, "message": f"navigating to {waypoint_name}"}

    @app.post("/cmd/cancel")
    async def cancel_command() -> dict:
        node.navigator_bridge.cancel()
        node._append_log("[Control] navigation cancel requested")
        return {"ok": True, "message": "navigation cancelled"}

    @app.post("/cmd/text")
    async def text_command(request: TextCommandRequest) -> dict:
        command = node.text_parser.parse(request.text)
        if command is None:
            raise HTTPException(status_code=400, detail="could not parse command")
        try:
            message = execute_parsed_command(node, command, source_prefix="text_command")
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"ok": True, "message": message}

    @app.post("/cmd/voice")
    async def voice_command() -> dict:
        if not node.voice_listener.available:
            raise HTTPException(status_code=400, detail=node.voice_listener.unavailable_reason)

        result = await run_in_threadpool(node.voice_listener.listen_once)
        if result.error:
            raise HTTPException(status_code=400, detail=result.error)

        command = node.text_parser.parse(result.text)
        if command is None:
            raise HTTPException(status_code=400, detail=f"could not parse voice command: {result.text}")

        try:
            message = await run_in_threadpool(
                execute_parsed_command,
                node,
                command,
                source_prefix="voice_command",
                prefer_voice_manual=True,
            )
        except (RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        node._append_log(f"[Voice] recognized: {result.text}")
        return {"ok": True, "message": message, "recognized_text": result.text}

    @app.post("/cmd/waypoint/save")
    async def save_waypoint(request: SaveWaypointRequest) -> dict:
        name = request.name.strip()
        if not name:
            raise HTTPException(status_code=400, detail="waypoint name cannot be empty")

        state = node.state_bridge.state
        if state.frame_id != "map":
            raise HTTPException(status_code=400, detail="map pose unavailable, waypoint save blocked")

        waypoint = node.waypoint_registry.save_waypoint(
            name=name,
            frame_id=state.frame_id,
            x=state.x,
            y=state.y,
            yaw_deg=math.degrees(state.yaw_rad),
        )
        node._append_log(
            f"[Control] save_waypoint: {waypoint.name} frame={waypoint.frame_id} "
            f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw_deg:.1f}"
        )
        return {"ok": True, "message": f"saved waypoint {waypoint.name}"}

    @app.post("/cmd/waypoint/delete")
    async def delete_waypoint(request: DeleteWaypointRequest) -> dict:
        waypoint_name = request.name.strip()
        if not waypoint_name:
            raise HTTPException(status_code=400, detail="waypoint name cannot be empty")
        deleted = node.waypoint_registry.delete_waypoint(waypoint_name)
        if not deleted:
            raise HTTPException(status_code=404, detail=f"unknown waypoint: {waypoint_name}")
        node._append_log(f"[Control] delete_waypoint: {waypoint_name}")
        return {"ok": True, "message": f"deleted waypoint {waypoint_name}"}

    @app.post("/cmd/waypoint/rename")
    async def rename_waypoint(request: RenameWaypointRequest) -> dict:
        try:
            waypoint = node.waypoint_registry.rename_waypoint(request.old_name, request.new_name)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        node._append_log(f"[Control] rename_waypoint: {request.old_name} -> {waypoint.name}")
        return {"ok": True, "message": f"renamed waypoint to {waypoint.name}"}

    @app.get("/telemetry/joints")
    async def telemetry_joints() -> dict:
        with node._lock:
            names = node.telemetry_bridge.joint_names()
        return {"names": names}

    @app.get("/telemetry/joint")
    async def telemetry_joint(name: str = "", type: str = "position") -> dict:
        import time as _time
        now_sec = _time.monotonic()
        with node._lock:
            if type == "velocity":
                times, values = node.telemetry_bridge.get_joint_velocity_series(name, now_sec)
            else:
                times, values = node.telemetry_bridge.get_joint_position_series(name, now_sec)
        return {"times": times, "values": values}

    @app.get("/telemetry/speed")
    async def telemetry_speed() -> dict:
        import time as _time
        now_sec = _time.monotonic()
        with node._lock:
            cmd_lin_t, cmd_lin_v = node.telemetry_bridge.get_cmd_linear_series(now_sec)
            cmd_ang_t, cmd_ang_v = node.telemetry_bridge.get_cmd_angular_series(now_sec)
            odom_lin_t, odom_lin_v = node.telemetry_bridge.get_odom_linear_series(now_sec)
            odom_ang_t, odom_ang_v = node.telemetry_bridge.get_odom_angular_series(now_sec)
        return {
            "cmd_linear": {"times": cmd_lin_t, "values": cmd_lin_v},
            "cmd_angular": {"times": cmd_ang_t, "values": cmd_ang_v},
            "odom_linear": {"times": odom_lin_t, "values": odom_lin_v},
            "odom_angular": {"times": odom_ang_t, "values": odom_ang_v},
        }

    @app.post("/stack/start")
    async def stack_start(request: StackTargetRequest) -> dict:
        ok, message = node.launch_manager.start(request.target)
        if not ok:
            raise HTTPException(status_code=400, detail=message)
        return {"ok": True, "message": message}

    @app.post("/stack/stop")
    async def stack_stop(request: StackTargetRequest) -> dict:
        ok, message = node.launch_manager.stop(request.target)
        if not ok:
            raise HTTPException(status_code=400, detail=message)
        return {"ok": True, "message": message}

    @app.post("/stack/rviz/open")
    async def rviz_open() -> dict:
        ok, message = node.launch_manager.open_rviz()
        if not ok:
            raise HTTPException(status_code=400, detail=message)
        return {"ok": True, "message": message}

    @app.post("/stack/rviz/close")
    async def rviz_close() -> dict:
        ok, message = node.launch_manager.close_rviz()
        if not ok:
            raise HTTPException(status_code=400, detail=message)
        return {"ok": True, "message": message}

    return app


def main() -> None:
    share_dir = Path(get_package_share_directory("go2_gui_controller"))
    waypoint_file = share_dir / "config" / "waypoints.yaml"
    index_file = share_dir / "web" / "index.html"

    rclpy.init()
    node = WebControllerNode(waypoint_file)
    node.navigator_bridge.start_wait_until_active()
    spin_thread = threading.Thread(target=_ros_spin_loop, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node, index_file)
    host = str(node.get_parameter("host").value or "127.0.0.1")
    port = int(node.get_parameter("port").value or 8080)

    try:
        uvicorn.run(app, host=host, port=port)
    finally:
        node.stop_runtime()
        node.manual_bridge.stop()
        node.navigator_bridge.cancel()
        node.launch_manager.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
