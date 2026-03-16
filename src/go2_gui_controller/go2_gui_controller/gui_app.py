from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import (
        QApplication,
        QDialog,
        QFrame,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QPushButton,
        QStackedWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QDialog,
        QFrame,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QPushButton,
        QStackedWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

from .charts_panel import ChartsPanel
from .commands import CommandType, ParsedCommand
from .launch_manager import LaunchManager
from .manual_control import ManualControlBridge
from .navigator_bridge import NavigatorBridge
from .state_bridge import StateBridge
from .telemetry_bridge import TelemetryBridge
from .text_command_parser import TextCommandParser
from .voice_command_listener import VoiceCommandListener, VoiceRecognitionResult
from .waypoint_registry import WaypointRegistry


@dataclass(frozen=True)
class RuntimeMode:
    key: str
    label: str
    use_sim_time: bool
    odom_topic: str


SIM_MODE = RuntimeMode("sim", "Simulation", True, "/odom")
REAL_MODE = RuntimeMode("real", "Real Robot", False, "/utlidar/robot_odom")


class ModeSelectionDialog(QDialog):
    def __init__(self):
        super().__init__()
        self._selected_mode: RuntimeMode | None = None
        self.setWindowTitle("Select Runtime Mode")
        self.setModal(True)
        self.setMinimumWidth(420)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(14)

        title = QLabel("Choose startup mode")
        title.setStyleSheet("font-size: 20px; font-weight: 700; color: #102a24;")
        hint = QLabel(
            "ROS connections start after this choice, so the GUI binds to the correct "
            "clock and odometry topic from the beginning."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #48645c; font-size: 13px;")

        sim_button = QPushButton("Simulation")
        sim_button.clicked.connect(lambda: self._select_mode(SIM_MODE))
        real_button = QPushButton("Real Robot")
        real_button.clicked.connect(lambda: self._select_mode(REAL_MODE))
        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)

        for button in (sim_button, real_button, cancel_button):
            button.setMinimumHeight(44)

        layout.addWidget(title)
        layout.addWidget(hint)
        layout.addWidget(sim_button)
        layout.addWidget(real_button)
        layout.addWidget(cancel_button)

    @property
    def selected_mode(self) -> RuntimeMode | None:
        return self._selected_mode

    def _select_mode(self, mode: RuntimeMode) -> None:
        self._selected_mode = mode
        self.accept()


class GuiControllerNode(Node):
    def __init__(self, default_waypoint_file: Path, runtime_mode: RuntimeMode):
        super().__init__("go2_gui_controller")
        self._declare_or_set_parameter("mode", runtime_mode.key)
        self._declare_or_set_parameter("use_sim_time", runtime_mode.use_sim_time)
        self._declare_or_set_parameter("waypoint_file", str(default_waypoint_file))
        self._declare_or_set_parameter("odom_topic", runtime_mode.odom_topic)
        waypoint_file = Path(self.get_parameter("waypoint_file").value)
        self.waypoint_registry = WaypointRegistry(waypoint_file)
        self.state_bridge = StateBridge(self, odom_topic=runtime_mode.odom_topic)
        self.telemetry = TelemetryBridge(self, odom_topic=runtime_mode.odom_topic)
        self.manual_control = ManualControlBridge(self)
        self.navigator = NavigatorBridge(self, self.state_bridge, self.waypoint_registry)
        self.parser = TextCommandParser(self.waypoint_registry)
        self.runtime_mode = runtime_mode

    def _declare_or_set_parameter(self, name: str, value) -> None:
        if self.has_parameter(name):
            self.set_parameters([Parameter(name, value=value)])
            return
        self.declare_parameter(name, value)


class MainWindow(QMainWindow):
    MANUAL_LINEAR_X = 1.0
    MANUAL_LINEAR_Y = 0.6
    MANUAL_ANGULAR_Z = 1.0
    VOICE_LINEAR_X = 0.35
    VOICE_LINEAR_Y = 0.25
    VOICE_ANGULAR_Z = 0.8
    VOICE_JOG_SEC = 0.8
    VOICE_FINE_JOG_SEC = 0.4
    VOICE_POLL_MS = 100
    ROS_SPIN_MS = 20
    ROS_SPIN_BATCH = 25
    STATUS_REFRESH_MS = 50
    CHART_REFRESH_MS = 100
    MANUAL_PUBLISH_MS = 50

    def __init__(self, ros_node: GuiControllerNode):
        super().__init__()
        self._node = ros_node
        self._launch_manager: LaunchManager | None = None
        self._manual_command = (0.0, 0.0, 0.0)
        self._timed_command = (0.0, 0.0, 0.0)
        self._timed_command_deadline = 0.0
        self._timed_command_source = ""
        self._voice_listener = VoiceCommandListener()
        self._voice_result_queue: queue.Queue[VoiceRecognitionResult] = queue.Queue()
        self._voice_listening = False
        self.setWindowTitle(f"Go2 Operator Console [{self._node.runtime_mode.label}]")
        self._build_ui()
        self._setup_runtime_launch_manager()

        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._spin_ros)
        self._ros_timer.start(self.ROS_SPIN_MS)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(self.STATUS_REFRESH_MS)

        self._chart_timer = QTimer(self)
        self._chart_timer.timeout.connect(self._refresh_charts)
        self._chart_timer.start(self.CHART_REFRESH_MS)

        self._manual_timer = QTimer(self)
        self._manual_timer.timeout.connect(self._publish_motion_command)
        self._manual_timer.start(self.MANUAL_PUBLISH_MS)

        self._voice_timer = QTimer(self)
        self._voice_timer.timeout.connect(self._poll_voice_results)
        self._voice_timer.start(self.VOICE_POLL_MS)

    def _build_ui(self) -> None:
        self._apply_styles()
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(16)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("Command and activity log")
        self.log_view.setObjectName("ConsoleLog")

        self.feedback_label = QLabel("Feedback: ready")
        self.voice_status_label = QLabel(self._voice_status_text())
        self.feedback_label.setObjectName("FeedbackBanner")
        self.feedback_label.setWordWrap(True)

        layout.addWidget(self._build_header())

        shell_row = QHBoxLayout()
        shell_row.setSpacing(16)
        self.page_stack = QStackedWidget()
        shell_row.addWidget(self._build_sidebar(), 0)

        self.page_stack.addWidget(self._build_control_tab())
        self.page_stack.addWidget(self._build_runtime_tab())
        self._charts_panel = ChartsPanel(self._node.telemetry)
        self.page_stack.addWidget(self._charts_panel)
        self.page_stack.addWidget(self._build_logs_tab())
        shell_row.addWidget(self.page_stack, 1)
        layout.addLayout(shell_row, 1)

        self.nav_list.setCurrentRow(0)
        self.page_stack.setCurrentIndex(0)

        self.setCentralWidget(root)

    def _apply_styles(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background: #f4efe5;
                color: #14342d;
                font-size: 13px;
            }
            QMainWindow {
                background: #efe7da;
            }
            QFrame#HeaderCard, QFrame#SidebarCard, QFrame#PanelCard, QFrame#MetricCard {
                background: #fbf8f1;
                border: 1px solid #d9cebd;
                border-radius: 18px;
            }
            QLabel#Eyebrow {
                color: #8b5e34;
                font-size: 11px;
                font-weight: 700;
                letter-spacing: 1px;
                text-transform: uppercase;
            }
            QLabel#PageTitle {
                font-size: 30px;
                font-weight: 800;
                color: #102a24;
            }
            QLabel#PageSubtitle {
                color: #48645c;
                font-size: 13px;
            }
            QLabel#SectionTitle {
                font-size: 18px;
                font-weight: 700;
                color: #102a24;
            }
            QLabel#SectionHint {
                color: #5f766f;
                font-size: 12px;
            }
            QLabel#MetricTitle {
                color: #7b6754;
                font-size: 11px;
                font-weight: 700;
                text-transform: uppercase;
            }
            QLabel#MetricValue {
                color: #14342d;
                font-size: 16px;
                font-weight: 700;
            }
            QLabel#FeedbackBanner {
                background: #163c34;
                color: #f8f3e7;
                border-radius: 12px;
                padding: 10px 12px;
                font-weight: 600;
            }
            QListWidget#SidebarNav {
                background: transparent;
                border: none;
                outline: none;
                padding: 4px;
            }
            QListWidget#SidebarNav::item {
                background: transparent;
                border-radius: 12px;
                padding: 12px 14px;
                margin: 4px 0px;
                color: #24463e;
                font-weight: 600;
            }
            QListWidget#SidebarNav::item:selected {
                background: #14342d;
                color: #f7f1e3;
            }
            QPushButton {
                background: #1d5448;
                color: #f8f3e7;
                border: none;
                border-radius: 12px;
                padding: 10px 14px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #246757;
            }
            QPushButton:pressed {
                background: #143d34;
            }
            QLineEdit, QTextEdit, QListWidget {
                background: #fffdfa;
                border: 1px solid #d7cab8;
                border-radius: 12px;
                padding: 8px 10px;
            }
            QTextEdit#ConsoleLog {
                background: #fffdfa;
                border-radius: 16px;
                padding: 12px;
            }
            """
        )

    def _build_header(self) -> QWidget:
        card = QFrame()
        card.setObjectName("HeaderCard")
        layout = QHBoxLayout(card)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(20)

        title_col = QVBoxLayout()
        eyebrow = QLabel("Operator Console")
        eyebrow.setObjectName("Eyebrow")
        title = QLabel("Go2 Mission Desk")
        title.setObjectName("PageTitle")
        subtitle = QLabel("Control, observe, and debug the robot from one focused workspace.")
        subtitle.setObjectName("PageSubtitle")
        subtitle.setWordWrap(True)
        title_col.addWidget(eyebrow)
        title_col.addWidget(title)
        title_col.addWidget(subtitle)
        layout.addLayout(title_col, 1)

        metrics_col = QHBoxLayout()
        metrics_col.setSpacing(12)
        nav_card, self.header_nav_value = self._make_metric_card("Nav Status", "idle")
        voice_card, self.header_voice_value = self._make_metric_card(
            "Voice",
            self._voice_status_text().replace("Voice: ", ""),
        )
        pose_card, self.header_pose_value = self._make_metric_card("Pose Frame", "odom")
        metrics_col.addWidget(self._wrap_metric(nav_card))
        metrics_col.addWidget(self._wrap_metric(voice_card))
        metrics_col.addWidget(self._wrap_metric(pose_card))
        layout.addLayout(metrics_col)
        return card

    def _build_sidebar(self) -> QWidget:
        card = QFrame()
        card.setObjectName("SidebarCard")
        card.setFixedWidth(220)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)

        section = QLabel("Views")
        section.setObjectName("Eyebrow")
        label = QLabel("Console Panels")
        label.setObjectName("SectionTitle")
        hint = QLabel("Popular robotics tools tend to keep navigation simple and persistent. This follows that pattern.")
        hint.setObjectName("SectionHint")
        hint.setWordWrap(True)

        self.nav_list = QListWidget()
        self.nav_list.setObjectName("SidebarNav")
        self.nav_list.addItems(["Control", "Runtime", "Charts", "Logs"])
        self.nav_list.currentRowChanged.connect(self.page_stack.setCurrentIndex)

        layout.addWidget(section)
        layout.addWidget(label)
        layout.addWidget(hint)
        layout.addWidget(self.nav_list, 1)
        return card

    def _build_panel_card(self, title: str, hint: str | None = None) -> tuple[QFrame, QVBoxLayout]:
        card = QFrame()
        card.setObjectName("PanelCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        header = QLabel(title)
        header.setObjectName("SectionTitle")
        layout.addWidget(header)
        if hint:
            hint_label = QLabel(hint)
            hint_label.setObjectName("SectionHint")
            hint_label.setWordWrap(True)
            layout.addWidget(hint_label)
        return card, layout

    def _make_metric_card(self, title: str, value: str) -> tuple[QFrame, QLabel]:
        card = QFrame()
        card.setObjectName("MetricCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)
        title_label = QLabel(title)
        title_label.setObjectName("MetricTitle")
        value_label = QLabel(value)
        value_label.setObjectName("MetricValue")
        value_label.setWordWrap(True)
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        return card, value_label

    def _wrap_metric(self, card: QWidget) -> QWidget:
        card.setMinimumWidth(150)
        return card

    def _build_control_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        layout.addWidget(self.feedback_label)

        top_row = QHBoxLayout()
        top_row.setSpacing(16)

        manual_card, manual_layout = self._build_panel_card(
            "Manual Control",
            "Low-latency nudges and stop-first control for close-range adjustment.",
        )

        manual_grid = QGridLayout()
        manual_grid.setSpacing(10)
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
        manual_layout.addLayout(manual_grid)
        top_row.addWidget(manual_card, 1)

        waypoint_card, waypoint_layout = self._build_panel_card(
            "Waypoints",
            "Save current map pose, rename it, and send RViz-compatible navigation goals.",
        )

        self.pose_label = QLabel("Pose: x=0.00, y=0.00")
        self.nav_status_label = QLabel("Nav: idle")
        self.last_result_label = QLabel("Last result: none")

        self.waypoint_list = QListWidget()
        self._reload_waypoint_list()
        waypoint_layout.addWidget(self.waypoint_list)

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
        waypoint_layout.addLayout(waypoint_row)

        save_row = QHBoxLayout()
        self.save_name_input = QLineEdit()
        self.save_name_input.setPlaceholderText("Waypoint name to save current pose")
        save_button = QPushButton("Save Current Pose")
        save_button.clicked.connect(self._save_current_pose)
        save_row.addWidget(self.save_name_input)
        save_row.addWidget(save_button)
        waypoint_layout.addLayout(save_row)

        rename_row = QHBoxLayout()
        self.rename_name_input = QLineEdit()
        self.rename_name_input.setPlaceholderText("New name for selected waypoint")
        rename_button = QPushButton("Rename Waypoint")
        rename_button.clicked.connect(self._rename_selected_waypoint)
        rename_row.addWidget(self.rename_name_input)
        rename_row.addWidget(rename_button)
        waypoint_layout.addLayout(rename_row)
        top_row.addWidget(waypoint_card, 1)
        layout.addLayout(top_row)

        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(16)

        text_card, text_layout = self._build_panel_card(
            "Text Command",
            "Structured command parsing for waypoint and relative navigation.",
        )
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Type commands like: go to home, forward 1m, turn left 90")
        run_button = QPushButton("Run Text Command")
        run_button.clicked.connect(self._run_text_command)
        text_layout.addWidget(self.command_input)
        text_layout.addWidget(run_button)
        bottom_row.addWidget(text_card, 1)

        voice_card, voice_layout = self._build_panel_card(
            "Voice Command",
            "Quick speech-triggered actions with fallback transcript testing.",
        )
        voice_layout.addWidget(self.voice_status_label)

        voice_row = QHBoxLayout()
        self.voice_transcript_input = QLineEdit()
        self.voice_transcript_input.setPlaceholderText("Future voice transcript or manual test input")
        voice_run_button = QPushButton("Run Voice Route")
        voice_run_button.clicked.connect(self._run_voice_transcript_input)
        self.voice_button = QPushButton("Listen Voice Command")
        self.voice_button.clicked.connect(self._start_voice_command)
        self.voice_button.setEnabled(self._voice_listener.available)
        voice_row.addWidget(self.voice_transcript_input)
        voice_row.addWidget(voice_run_button)
        voice_row.addWidget(self.voice_button)
        voice_layout.addLayout(voice_row)
        bottom_row.addWidget(voice_card, 1)

        layout.addLayout(bottom_row)
        return tab

    def _build_runtime_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        runtime_card, runtime_layout = self._build_panel_card(
            "Runtime Launch",
            "Start and stop the SLAM mapping stack or the localization plus Nav2 stack from the GUI.",
        )
        self.slam_stack_status_label = QLabel("SLAM process: unavailable")
        self.slam_ready_label = QLabel("SLAM ready: unavailable")
        self.navigation_stack_status_label = QLabel("Navigation process: unavailable")
        self.navigation_ready_label = QLabel("Navigation ready: unavailable")
        self.rviz_status_label = QLabel("RViz: unavailable")
        self.runtime_launch_hint_label = QLabel("Launch controls are initializing.")
        self.tf_debug_label = QLabel("TF debug: unavailable")
        self.tf_debug_label.setWordWrap(True)
        self.runtime_launch_hint_label.setWordWrap(True)
        runtime_layout.addWidget(self.slam_stack_status_label)
        runtime_layout.addWidget(self.slam_ready_label)
        runtime_layout.addWidget(self.navigation_stack_status_label)
        runtime_layout.addWidget(self.navigation_ready_label)
        runtime_layout.addWidget(self.rviz_status_label)
        runtime_layout.addWidget(self.runtime_launch_hint_label)
        runtime_layout.addWidget(self.tf_debug_label)

        runtime_button_row = QHBoxLayout()
        self.start_slam_button = QPushButton("Start SLAM")
        self.stop_slam_button = QPushButton("Stop SLAM")
        self.start_navigation_button = QPushButton("Start Navigation")
        self.stop_navigation_button = QPushButton("Stop Navigation")
        runtime_button_row.addWidget(self.start_slam_button)
        runtime_button_row.addWidget(self.stop_slam_button)
        runtime_button_row.addWidget(self.start_navigation_button)
        runtime_button_row.addWidget(self.stop_navigation_button)
        runtime_layout.addLayout(runtime_button_row)

        rviz_button_row = QHBoxLayout()
        self.open_rviz_button = QPushButton("Open RViz")
        self.close_rviz_button = QPushButton("Close RViz")
        rviz_button_row.addWidget(self.open_rviz_button)
        rviz_button_row.addWidget(self.close_rviz_button)
        runtime_layout.addLayout(rviz_button_row)

        layout.addWidget(runtime_card)
        layout.addStretch(1)
        return tab

    def _build_logs_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)

        intro_card, intro_layout = self._build_panel_card(
            "Event Log",
            "Chronological command, parser, waypoint, and runtime events for quick triage.",
        )
        intro_layout.addWidget(self.log_view)
        layout.addWidget(intro_card)
        return tab

    def _setup_runtime_launch_manager(self) -> None:
        self._launch_manager = LaunchManager(
            self._node.runtime_mode.key,
            log_callback=self._append_log,
            status_callback=self._set_runtime_stack_status,
        )

        self.start_slam_button.clicked.connect(self._start_slam_stack)
        self.stop_slam_button.clicked.connect(self._stop_slam_stack)
        self.start_navigation_button.clicked.connect(self._start_navigation_stack)
        self.stop_navigation_button.clicked.connect(self._stop_navigation_stack)
        self.open_rviz_button.clicked.connect(self._open_rviz)
        self.close_rviz_button.clicked.connect(self._close_rviz)

        if not self._launch_manager.available:
            self.runtime_launch_hint_label.setText(
                f"Launch controls unavailable: {self._launch_manager.unavailable_reason}"
            )
            for button in (
                self.start_slam_button,
                self.stop_slam_button,
                self.start_navigation_button,
                self.stop_navigation_button,
                self.open_rviz_button,
                self.close_rviz_button,
            ):
                button.setEnabled(False)
            self._append_log(f"runtime_launch unavailable: {self._launch_manager.unavailable_reason}")
            return

        mode_text = "simulation" if self._node.runtime_mode.key == "sim" else "real robot"
        self.runtime_launch_hint_label.setText(
            f"Launch targets are bound to the {mode_text} stack for this session."
        )
        self._set_runtime_stack_status("slam", self._launch_manager.status_text("slam"))
        self._set_runtime_stack_status("navigation", self._launch_manager.status_text("navigation"))
        self._set_runtime_stack_status("rviz", self._launch_manager.rviz_status_text())
        self._refresh_runtime_ready_labels()

    def _set_runtime_stack_status(self, key: str, status: str) -> None:
        if key == "slam":
            label = self.slam_stack_status_label
            prefix = "SLAM process"
        elif key == "navigation":
            label = self.navigation_stack_status_label
            prefix = "Navigation process"
        else:
            label = self.rviz_status_label
            prefix = "RViz"
        label.setText(f"{prefix}: {status}")

    def _start_slam_stack(self) -> None:
        self._run_runtime_stack_action("slam", "start")

    def _stop_slam_stack(self) -> None:
        self._run_runtime_stack_action("slam", "stop")

    def _start_navigation_stack(self) -> None:
        self._run_runtime_stack_action("navigation", "start")

    def _stop_navigation_stack(self) -> None:
        self._run_runtime_stack_action("navigation", "stop")

    def _open_rviz(self) -> None:
        self._run_runtime_stack_action("rviz", "start")

    def _close_rviz(self) -> None:
        self._run_runtime_stack_action("rviz", "stop")

    def _run_runtime_stack_action(self, key: str, action: str) -> None:
        if self._launch_manager is None:
            self.feedback_label.setText("Feedback: launch manager unavailable")
            self._append_log("runtime_launch failed: manager unavailable")
            return

        if key == "rviz":
            if action == "start":
                success, message = self._launch_manager.open_rviz()
            else:
                success, message = self._launch_manager.close_rviz()
        elif action == "start":
            success, message = self._launch_manager.start(key)
        else:
            success, message = self._launch_manager.stop(key)

        if success and key == "navigation" and action == "start":
            self._node.state_bridge.reset_localization_tracking()

        self.feedback_label.setText(f"Feedback: {message}")
        log_prefix = "runtime_launch" if success else "runtime_launch blocked"
        self._append_log(f"{log_prefix}: {message}")

    def _spin_ros(self) -> None:
        for _ in range(self.ROS_SPIN_BATCH):
            rclpy.spin_once(self._node, timeout_sec=0.0)
        self._node.state_bridge.update_from_tf()
        self._node.navigator.spin_once()

    def _refresh_status(self) -> None:
        state = self._node.state_bridge.state
        pose_text = self._node.state_bridge.pose_text()
        nav_status_text = self._display_nav_status(state.nav_status)
        self.pose_label.setText(f"Pose: {pose_text}")
        self.nav_status_label.setText(f"Nav: {nav_status_text}")
        self.last_result_label.setText(f"Last result: {state.last_result}")
        self.header_nav_value.setText(nav_status_text)
        self.header_voice_value.setText(self.voice_status_label.text().replace("Voice: ", ""))
        self.header_pose_value.setText(state.frame_id)
        self._refresh_runtime_ready_labels()

    def _refresh_charts(self) -> None:
        self._charts_panel.refresh()

    def _display_nav_status(self, raw_status: str) -> str:
        if self._launch_manager is None or not self._launch_manager.available:
            return raw_status
        if not self._launch_manager.is_process_running("navigation"):
            return "inactive"
        return raw_status

    def _refresh_runtime_ready_labels(self) -> None:
        if self._launch_manager is None or not self._launch_manager.available:
            return
        self.slam_ready_label.setText(f"SLAM ready: {self._compute_slam_ready_text()}")
        self.navigation_ready_label.setText(f"Navigation ready: {self._compute_navigation_ready_text()}")
        self.tf_debug_label.setText(self._compute_tf_debug_text())

    def _compute_slam_ready_text(self) -> str:
        if not self._launch_manager.is_process_running("slam"):
            return "inactive"

        issues = []
        odom_topic = self._node.runtime_mode.odom_topic
        if not self._node.telemetry.has_recent_topic_data(odom_topic, 1.5):
            issues.append("waiting for odom")
        if not self._node.telemetry.has_recent_topic_data("/scan", 1.5):
            issues.append("waiting for scan")
        if not self._node.telemetry.has_topic_data("/map"):
            issues.append("waiting for first map")
        return "ready" if not issues else ", ".join(issues)

    def _compute_navigation_ready_text(self) -> str:
        if not self._launch_manager.is_process_running("navigation"):
            return "inactive"

        issues = []
        odom_topic = self._node.runtime_mode.odom_topic
        if not self._node.telemetry.has_recent_topic_data(odom_topic, 1.5):
            issues.append("waiting for odom")
        if not self._node.telemetry.has_topic_data("/map"):
            issues.append("waiting for map")
        if not self._node.navigator.server_ready():
            issues.append("waiting for navigate_to_pose")
        if not self._node.navigator.localization_ready():
            issues.append("waiting for localization")
        return "ready" if not issues else ", ".join(issues)

    def _compute_tf_debug_text(self) -> str:
        state = self._node.state_bridge.state
        map_odom_text = "yes" if state.map_odom_seen else "no"
        map_base_text = "yes" if state.map_base_link_seen else "no"
        map_camera_text = "yes" if state.map_camera_link_seen else "no"
        localization_text = "yes" if state.localization_initialized else "no"
        return (
            "TF debug: "
            f"map->odom {map_odom_text}, "
            f"map->base_link {map_base_text}, "
            f"map->camera_link {map_camera_text}, "
            f"localization initialized {localization_text}, "
            f"pose frame {state.frame_id}"
        )

    def _reload_waypoint_list(self) -> None:
        self.waypoint_list.clear()
        for waypoint in self._node.waypoint_registry.list_waypoints():
            self.waypoint_list.addItem(QListWidgetItem(waypoint.name))
        self._append_log("waypoint list refreshed")

    def _append_log(self, message: str) -> None:
        self.log_view.append(message)

    def _voice_status_text(self) -> str:
        if self._voice_listener.available:
            return "Voice: ready"
        return f"Voice: unavailable ({self._voice_listener.unavailable_reason})"

    def _start_voice_command(self) -> None:
        if self._voice_listening:
            return
        if not self._voice_listener.available:
            self.voice_status_label.setText(self._voice_status_text())
            self.feedback_label.setText("Feedback: voice backend unavailable")
            self._append_log("voice_command blocked: backend unavailable")
            return
        self._voice_listening = True
        self.voice_button.setEnabled(False)
        self.voice_button.setText("Listening...")
        self.voice_status_label.setText("Voice: listening")
        self.voice_transcript_input.setText("")
        worker = threading.Thread(target=self._listen_voice_worker, daemon=True)
        worker.start()

    def _listen_voice_worker(self) -> None:
        result = self._voice_listener.listen_once()
        self._voice_result_queue.put(result)

    def _poll_voice_results(self) -> None:
        while True:
            try:
                result = self._voice_result_queue.get_nowait()
            except queue.Empty:
                return
            self._finish_voice_command(result)

    def _finish_voice_command(self, result: VoiceRecognitionResult) -> None:
        self._voice_listening = False
        self.voice_button.setEnabled(self._voice_listener.available)
        self.voice_button.setText("Listen Voice Command")
        self.voice_status_label.setText(self._voice_status_text())
        if result.error:
            self.feedback_label.setText(f"Feedback: voice command failed ({result.error})")
            self._append_log(f"voice_command failed: {result.error}")
            return
        self.voice_transcript_input.setText(result.text)
        self._append_log(f"voice_command recognized: {result.text}")
        self._run_voice_command(result.text)

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
        self._clear_timed_command()
        self._node.navigator.cancel()
        self.feedback_label.setText("Feedback: cancel requested")
        self._append_log("cancel_navigation requested")

    def _bind_hold_button(self, button: QPushButton, linear_x: float, linear_y: float, angular_z: float) -> None:
        button.pressed.connect(
            lambda lx=linear_x, ly=linear_y, az=angular_z: self._start_manual_hold(lx, ly, az)
        )
        button.released.connect(self._release_manual_hold)

    def _start_manual_hold(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._clear_timed_command(stop_robot=False)
        self._node.navigator.cancel()
        self._manual_command = (linear_x, linear_y, angular_z)
        self._publish_motion_command()
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

    def _clear_timed_command(self, stop_robot: bool = True) -> None:
        was_active = self._timed_command_deadline > 0.0
        self._timed_command = (0.0, 0.0, 0.0)
        self._timed_command_deadline = 0.0
        self._timed_command_source = ""
        if was_active and stop_robot:
            self._node.manual_control.stop()

    def _publish_motion_command(self) -> None:
        linear_x, linear_y, angular_z = self._manual_command
        if linear_x != 0.0 or linear_y != 0.0 or angular_z != 0.0:
            self._node.manual_control.send_velocity(linear_x, linear_y, angular_z)
            return
        if self._timed_command_deadline <= 0.0:
            return
        if time.monotonic() >= self._timed_command_deadline:
            source = self._timed_command_source
            self._clear_timed_command()
            self.feedback_label.setText(f"Feedback: {source} complete")
            self._append_log(f"{source} complete")
            return
        self._node.manual_control.send_velocity(*self._timed_command)

    def _start_timed_command(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_sec: float,
        source: str,
    ) -> None:
        self._clear_manual_hold()
        self._clear_timed_command(stop_robot=False)
        self._node.navigator.cancel()
        self._timed_command = (linear_x, linear_y, angular_z)
        self._timed_command_deadline = time.monotonic() + max(duration_sec, 0.1)
        self._timed_command_source = source
        self._publish_motion_command()
        self.feedback_label.setText(
            f"Feedback: {source} x={linear_x:.2f}, y={linear_y:.2f}, yaw={angular_z:.2f}"
        )
        self._append_log(
            f"{source}: x={linear_x:.2f}, y={linear_y:.2f}, yaw={angular_z:.2f}, "
            f"duration={duration_sec:.2f}s"
        )

    def _stop(self) -> None:
        self._clear_manual_hold()
        self._clear_timed_command(stop_robot=False)
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

    def _run_voice_transcript_input(self) -> None:
        self._run_voice_command(self.voice_transcript_input.text())

    def _run_voice_command(self, text: str) -> None:
        command = self._node.parser.parse(text)
        if command is None:
            self.feedback_label.setText("Feedback: could not parse voice command")
            self._append_log(f"voice_command parse failed: {text}")
            return
        self._append_log(f"voice_command parsed: {text}")
        if self._execute_voice_manual_command(command):
            return
        self._execute_command(command, source_prefix="voice_command")

    def _execute_voice_manual_command(self, command: ParsedCommand) -> bool:
        if command.command_type == CommandType.STOP:
            self._stop()
            return True
        if command.command_type == CommandType.CANCEL_NAVIGATION:
            self._cancel_navigation()
            return True
        if not self._voice_prefers_manual(command):
            return False
        source_text = command.source_text.lower()
        jog_sec = self.VOICE_FINE_JOG_SEC if any(token in source_text for token in ("조금", "살짝")) else self.VOICE_JOG_SEC
        if command.command_type == CommandType.MOVE_RELATIVE:
            linear_x = 0.0
            linear_y = 0.0
            if command.x_m > 0.0:
                linear_x = self.VOICE_LINEAR_X
            elif command.x_m < 0.0:
                linear_x = -self.VOICE_LINEAR_X
            elif command.y_m > 0.0:
                linear_y = self.VOICE_LINEAR_Y
            elif command.y_m < 0.0:
                linear_y = -self.VOICE_LINEAR_Y
            self._start_timed_command(linear_x, linear_y, 0.0, jog_sec, "voice cmd_vel")
            return True
        if command.command_type == CommandType.ROTATE_RELATIVE:
            angular_z = self.VOICE_ANGULAR_Z if command.yaw_deg >= 0.0 else -self.VOICE_ANGULAR_Z
            self._start_timed_command(0.0, 0.0, angular_z, jog_sec, "voice cmd_vel")
            return True
        return False

    def _voice_prefers_manual(self, command: ParsedCommand) -> bool:
        if command.command_type not in (CommandType.MOVE_RELATIVE, CommandType.ROTATE_RELATIVE):
            return False
        source_text = command.source_text.lower()
        if any(character.isdigit() for character in source_text):
            return False
        if any(token in source_text for token in ("미터", "meter", "meters", "cm", "도", "deg", "degree", "degrees", "반바퀴")):
            return False
        return True

    def _execute_command(self, command: ParsedCommand, source_prefix: str = "text_command") -> None:
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
                self._append_log(f"{source_prefix} waypoint blocked: {exc}")
                return
            self.feedback_label.setText(f"Feedback: navigating to {command.waypoint_name}")
            self._append_log(f"{source_prefix} navigate_to_waypoint: {command.waypoint_name}")
            return
        if command.command_type == CommandType.MOVE_RELATIVE:
            try:
                self._node.navigator.go_to_relative_pose(command.x_m, command.y_m, 0.0)
            except RuntimeError as exc:
                self.feedback_label.setText(f"Feedback: {exc}")
                self._append_log(f"{source_prefix} move blocked: {exc}")
                return
            self.feedback_label.setText(
                f"Feedback: relative move x={command.x_m:.2f}, y={command.y_m:.2f}"
            )
            self._append_log(
                f"{source_prefix} move_relative: x={command.x_m:.2f}, y={command.y_m:.2f}"
            )
            return
        if command.command_type == CommandType.ROTATE_RELATIVE:
            try:
                self._node.navigator.go_to_relative_pose(0.0, 0.0, command.yaw_deg)
            except RuntimeError as exc:
                self.feedback_label.setText(f"Feedback: {exc}")
                self._append_log(f"{source_prefix} rotate blocked: {exc}")
                return
            self.feedback_label.setText(f"Feedback: relative rotate {command.yaw_deg:.1f} deg")
            self._append_log(f"{source_prefix} rotate_relative: {command.yaw_deg:.1f} deg")

    def closeEvent(self, event) -> None:
        self._clear_manual_hold()
        self._clear_timed_command(stop_robot=False)
        self._node.manual_control.stop()
        self._node.navigator.cancel()
        if self._launch_manager is not None:
            self._launch_manager.shutdown()
        super().closeEvent(event)


def run_app(waypoint_file: Path) -> None:
    app = QApplication([])
    mode_dialog = ModeSelectionDialog()
    exec_method = getattr(mode_dialog, "exec", None)
    dialog_result = exec_method() if callable(exec_method) else mode_dialog.exec_()
    if dialog_result != QDialog.Accepted or mode_dialog.selected_mode is None:
        return

    runtime_mode = mode_dialog.selected_mode
    rclpy.init()
    node = GuiControllerNode(waypoint_file, runtime_mode)
    window = MainWindow(node)
    window.resize(980, 760)
    window.show()
    try:
        node.navigator.start_wait_until_active()
        app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
