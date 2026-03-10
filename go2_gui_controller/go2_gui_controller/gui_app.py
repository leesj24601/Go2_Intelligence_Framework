from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
    from python_qt_binding.QtCore import QTimer
    from python_qt_binding.QtWidgets import (
        QApplication,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

from .commands import CommandType, ParsedCommand
from .manual_control import ManualControlBridge
from .navigator_bridge import NavigatorBridge
from .state_bridge import StateBridge
from .text_command_parser import TextCommandParser
from .waypoint_registry import WaypointRegistry


class GuiControllerNode(Node):
    def __init__(self, default_waypoint_file: Path):
        super().__init__("go2_gui_controller")
        if self.has_parameter("use_sim_time"):
            if self.get_parameter("use_sim_time").value is not True:
                self.set_parameters([Parameter("use_sim_time", value=True)])
        else:
            self.declare_parameter("use_sim_time", True)
        if not self.has_parameter("waypoint_file"):
            self.declare_parameter("waypoint_file", str(default_waypoint_file))
        waypoint_file = Path(self.get_parameter("waypoint_file").value)
        self.waypoint_registry = WaypointRegistry(waypoint_file)
        self.state_bridge = StateBridge(self)
        self.manual_control = ManualControlBridge(self)
        self.navigator = NavigatorBridge(self, self.state_bridge, self.waypoint_registry)
        self.parser = TextCommandParser(self.waypoint_registry)


class MainWindow(QMainWindow):
    MANUAL_LINEAR_X = 1.0
    MANUAL_LINEAR_Y = 0.6
    MANUAL_ANGULAR_Z = 1.0
    ROS_SPIN_MS = 20
    STATUS_REFRESH_MS = 50
    MANUAL_PUBLISH_MS = 50

    def __init__(self, ros_node: GuiControllerNode):
        super().__init__()
        self._node = ros_node
        self._manual_command = (0.0, 0.0, 0.0)
        self.setWindowTitle("Go2 GUI Controller")
        self._build_ui()

        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._spin_ros)
        self._ros_timer.start(self.ROS_SPIN_MS)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(self.STATUS_REFRESH_MS)

        self._manual_timer = QTimer(self)
        self._manual_timer.timeout.connect(self._publish_manual_hold)
        self._manual_timer.start(self.MANUAL_PUBLISH_MS)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)

        self.pose_label = QLabel("Pose: x=0.00, y=0.00")
        self.nav_status_label = QLabel("Nav: idle")
        self.last_result_label = QLabel("Last result: none")
        layout.addWidget(self.pose_label)
        layout.addWidget(self.nav_status_label)
        layout.addWidget(self.last_result_label)

        self.feedback_label = QLabel("Feedback: ready")
        layout.addWidget(self.feedback_label)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("Command and activity log")
        layout.addWidget(self.log_view)

        self.waypoint_list = QListWidget()
        self._reload_waypoint_list()
        layout.addWidget(self.waypoint_list)

        waypoint_row = QHBoxLayout()
        go_button = QPushButton("Go To Selected Waypoint")
        go_button.clicked.connect(self._go_to_selected_waypoint)
        delete_button = QPushButton("Delete Waypoint")
        delete_button.clicked.connect(self._delete_selected_waypoint)
        cancel_button = QPushButton("Cancel Goal")
        cancel_button.clicked.connect(self._cancel_navigation)
        waypoint_row.addWidget(go_button)
        waypoint_row.addWidget(delete_button)
        waypoint_row.addWidget(cancel_button)
        layout.addLayout(waypoint_row)

        save_row = QHBoxLayout()
        self.save_name_input = QLineEdit()
        self.save_name_input.setPlaceholderText("Waypoint name to save current pose")
        save_button = QPushButton("Save Current Pose")
        save_button.clicked.connect(self._save_current_pose)
        save_row.addWidget(self.save_name_input)
        save_row.addWidget(save_button)
        layout.addLayout(save_row)

        rename_row = QHBoxLayout()
        self.rename_name_input = QLineEdit()
        self.rename_name_input.setPlaceholderText("New name for selected waypoint")
        rename_button = QPushButton("Rename Waypoint")
        rename_button.clicked.connect(self._rename_selected_waypoint)
        rename_row.addWidget(self.rename_name_input)
        rename_row.addWidget(rename_button)
        layout.addLayout(rename_row)

        manual_grid = QGridLayout()
        forward_button = QPushButton("Forward")
        back_button = QPushButton("Back")
        left_button = QPushButton("Left")
        right_button = QPushButton("Right")
        turn_left_button = QPushButton("Turn Left")
        turn_right_button = QPushButton("Turn Right")
        stop_button = QPushButton("Stop")

        self._bind_hold_button(forward_button, self.MANUAL_LINEAR_X, 0.0, 0.0)
        self._bind_hold_button(back_button, -self.MANUAL_LINEAR_X, 0.0, 0.0)
        self._bind_hold_button(left_button, 0.0, self.MANUAL_LINEAR_Y, 0.0)
        self._bind_hold_button(right_button, 0.0, -self.MANUAL_LINEAR_Y, 0.0)
        self._bind_hold_button(turn_left_button, 0.0, 0.0, self.MANUAL_ANGULAR_Z)
        self._bind_hold_button(turn_right_button, 0.0, 0.0, -self.MANUAL_ANGULAR_Z)
        stop_button.clicked.connect(self._stop)

        manual_grid.addWidget(forward_button, 0, 1)
        manual_grid.addWidget(left_button, 1, 0)
        manual_grid.addWidget(stop_button, 1, 1)
        manual_grid.addWidget(right_button, 1, 2)
        manual_grid.addWidget(back_button, 2, 1)
        manual_grid.addWidget(turn_left_button, 3, 0)
        manual_grid.addWidget(turn_right_button, 3, 2)
        layout.addLayout(manual_grid)

        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Type commands like: go to home, forward 1m, turn left 90")
        run_button = QPushButton("Run Text Command")
        run_button.clicked.connect(self._run_text_command)
        layout.addWidget(self.command_input)
        layout.addWidget(run_button)

        self.setCentralWidget(root)

    def _spin_ros(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.0)
        self._node.state_bridge.update_from_tf()
        self._node.navigator.spin_once()

    def _refresh_status(self) -> None:
        state = self._node.state_bridge.state
        self.pose_label.setText(f"Pose: {self._node.state_bridge.pose_text()}")
        self.nav_status_label.setText(f"Nav: {state.nav_status}")
        self.last_result_label.setText(f"Last result: {state.last_result}")

    def _reload_waypoint_list(self) -> None:
        self.waypoint_list.clear()
        for waypoint in self._node.waypoint_registry.list_waypoints():
            self.waypoint_list.addItem(QListWidgetItem(waypoint.name))
        self._append_log("waypoint list refreshed")

    def _append_log(self, message: str) -> None:
        self.log_view.append(message)

    def _go_to_selected_waypoint(self) -> None:
        item = self.waypoint_list.currentItem()
        if item is None:
            self.feedback_label.setText("Feedback: select a waypoint first")
            self._append_log("go_to_waypoint failed: no waypoint selected")
            return
        waypoint_name = item.text()
        waypoint = self._node.waypoint_registry.get(waypoint_name)
        if waypoint is None:
            self.feedback_label.setText("Feedback: selected waypoint not found")
            self._append_log(f"go_to_waypoint failed: missing waypoint {waypoint_name}")
            return
        try:
            self._node.navigator.go_to_waypoint(waypoint_name)
        except RuntimeError as exc:
            self.feedback_label.setText(f"Feedback: {exc}")
            self._append_log(f"go_to_waypoint blocked: {exc}")
            return
        self.feedback_label.setText(
            f"Feedback: navigating to {waypoint_name} ({waypoint.x:.2f}, {waypoint.y:.2f})"
        )
        self._append_log(
            f"go_to_waypoint: {waypoint_name} frame={waypoint.frame_id} "
            f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw_deg:.1f}"
        )

    def _delete_selected_waypoint(self) -> None:
        item = self.waypoint_list.currentItem()
        if item is None:
            self.feedback_label.setText("Feedback: select a waypoint first")
            self._append_log("delete_waypoint failed: no waypoint selected")
            return
        waypoint_name = item.text()
        deleted = self._node.waypoint_registry.delete_waypoint(waypoint_name)
        if not deleted:
            self.feedback_label.setText("Feedback: failed to delete waypoint")
            self._append_log(f"delete_waypoint failed: {waypoint_name}")
            return
        self._reload_waypoint_list()
        self.feedback_label.setText(f"Feedback: deleted waypoint {waypoint_name}")
        self._append_log(f"delete_waypoint: {waypoint_name}")

    def _save_current_pose(self) -> None:
        name = self.save_name_input.text().strip()
        if not name:
            self.feedback_label.setText("Feedback: enter a waypoint name first")
            self._append_log("save_waypoint failed: empty name")
            return
        state = self._node.state_bridge.state
        if state.frame_id != "map":
            self.feedback_label.setText("Feedback: map pose unavailable, waypoint save blocked")
            self._append_log("save_waypoint blocked: current pose is not in map frame")
            return
        yaw_deg = state.yaw_rad * 180.0 / 3.141592653589793
        waypoint = self._node.waypoint_registry.save_waypoint(
            name=name,
            frame_id=state.frame_id,
            x=state.x,
            y=state.y,
            yaw_deg=yaw_deg,
        )
        self._reload_waypoint_list()
        self.feedback_label.setText(
            f"Feedback: saved waypoint {name} ({waypoint.x:.2f}, {waypoint.y:.2f})"
        )
        self._append_log(
            f"save_waypoint: {name} frame={waypoint.frame_id} "
            f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw_deg:.1f}"
        )

    def _rename_selected_waypoint(self) -> None:
        item = self.waypoint_list.currentItem()
        if item is None:
            self.feedback_label.setText("Feedback: select a waypoint first")
            self._append_log("rename_waypoint failed: no waypoint selected")
            return
        new_name = self.rename_name_input.text().strip()
        if not new_name:
            self.feedback_label.setText("Feedback: enter a new waypoint name")
            self._append_log("rename_waypoint failed: empty new name")
            return
        old_name = item.text()
        try:
            self._node.waypoint_registry.rename_waypoint(old_name, new_name)
        except ValueError as exc:
            self.feedback_label.setText(f"Feedback: {exc}")
            self._append_log(f"rename_waypoint failed: {exc}")
            return
        self._reload_waypoint_list()
        self.feedback_label.setText(f"Feedback: renamed {old_name} -> {new_name}")
        self._append_log(f"rename_waypoint: {old_name} -> {new_name}")

    def _cancel_navigation(self) -> None:
        self._clear_manual_hold()
        self._node.navigator.cancel()
        self.feedback_label.setText("Feedback: cancel requested")
        self._append_log("cancel_navigation requested")

    def _bind_hold_button(self, button: QPushButton, linear_x: float, linear_y: float, angular_z: float) -> None:
        button.pressed.connect(
            lambda lx=linear_x, ly=linear_y, az=angular_z: self._start_manual_hold(lx, ly, az)
        )
        button.released.connect(self._release_manual_hold)

    def _start_manual_hold(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._node.navigator.cancel()
        self._manual_command = (linear_x, linear_y, angular_z)
        self._publish_manual_hold()
        self.feedback_label.setText(
            f"Feedback: holding manual x={linear_x:.2f}, y={linear_y:.2f}, yaw={angular_z:.2f}"
        )
        self._append_log(
            f"manual_hold start: x={linear_x:.2f}, y={linear_y:.2f}, yaw={angular_z:.2f}"
        )

    def _release_manual_hold(self) -> None:
        self._clear_manual_hold()
        self._node.manual_control.stop()
        self.feedback_label.setText("Feedback: manual hold released")
        self._append_log("manual_hold released")

    def _clear_manual_hold(self) -> None:
        self._manual_command = (0.0, 0.0, 0.0)

    def _publish_manual_hold(self) -> None:
        linear_x, linear_y, angular_z = self._manual_command
        if linear_x == 0.0 and linear_y == 0.0 and angular_z == 0.0:
            return
        self._node.manual_control.send_velocity(linear_x, linear_y, angular_z)

    def _stop(self) -> None:
        self._clear_manual_hold()
        self._node.navigator.cancel()
        self._node.manual_control.stop()
        self.feedback_label.setText("Feedback: stop sent")
        self._append_log("stop sent")

    def _run_text_command(self) -> None:
        text = self.command_input.text()
        command = self._node.parser.parse(text)
        if command is None:
            self.feedback_label.setText("Feedback: could not parse command")
            self._append_log(f"text_command parse failed: {text}")
            return
        self._append_log(f"text_command parsed: {text}")
        self._execute_command(command)

    def _execute_command(self, command: ParsedCommand) -> None:
        if command.command_type == CommandType.STOP:
            self._stop()
            return
        if command.command_type == CommandType.CANCEL_NAVIGATION:
            self._cancel_navigation()
            return
        if command.command_type == CommandType.NAVIGATE_TO_WAYPOINT and command.waypoint_name:
            try:
                self._node.navigator.go_to_waypoint(command.waypoint_name)
            except RuntimeError as exc:
                self.feedback_label.setText(f"Feedback: {exc}")
                self._append_log(f"text_command waypoint blocked: {exc}")
                return
            self.feedback_label.setText(f"Feedback: navigating to {command.waypoint_name}")
            self._append_log(f"text_command navigate_to_waypoint: {command.waypoint_name}")
            return
        if command.command_type == CommandType.MOVE_RELATIVE:
            try:
                self._node.navigator.go_to_relative_pose(command.x_m, command.y_m, 0.0)
            except RuntimeError as exc:
                self.feedback_label.setText(f"Feedback: {exc}")
                self._append_log(f"text_command move blocked: {exc}")
                return
            self.feedback_label.setText(
                f"Feedback: relative move x={command.x_m:.2f}, y={command.y_m:.2f}"
            )
            self._append_log(
                f"text_command move_relative: x={command.x_m:.2f}, y={command.y_m:.2f}"
            )
            return
        if command.command_type == CommandType.ROTATE_RELATIVE:
            try:
                self._node.navigator.go_to_relative_pose(0.0, 0.0, command.yaw_deg)
            except RuntimeError as exc:
                self.feedback_label.setText(f"Feedback: {exc}")
                self._append_log(f"text_command rotate blocked: {exc}")
                return
            self.feedback_label.setText(f"Feedback: relative rotate {command.yaw_deg:.1f} deg")
            self._append_log(f"text_command rotate_relative: {command.yaw_deg:.1f} deg")

    def closeEvent(self, event) -> None:
        self._clear_manual_hold()
        self._node.manual_control.stop()
        self._node.navigator.cancel()
        super().closeEvent(event)


def run_app(waypoint_file: Path) -> None:
    rclpy.init()
    node = GuiControllerNode(waypoint_file)
    app = QApplication([])
    window = MainWindow(node)
    window.resize(520, 560)
    window.show()
    try:
        node.navigator.start_wait_until_active()
        app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
