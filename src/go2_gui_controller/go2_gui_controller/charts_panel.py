from __future__ import annotations

import time

try:
    from python_qt_binding.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget
except ImportError:
    from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

try:
    import pyqtgraph as pg
except Exception as exc:
    pg = None
    _pyqtgraph_import_error = str(exc)
else:
    _pyqtgraph_import_error = ""
    pg.setConfigOptions(antialias=True, background="#fbf8f1", foreground="#14342d")

from .telemetry_bridge import TelemetryBridge


class ChartsPanel(QWidget):
    def __init__(self, telemetry: TelemetryBridge):
        super().__init__()
        self._telemetry = telemetry
        self._joint_names: list[str] = []
        self._plot_widgets = []
        self._motion_placeholder_x = [-self._telemetry.history_sec, 0.0]
        self._motion_placeholder_y = [0.0, 0.0]

        layout = QVBoxLayout(self)

        self.summary_label = QLabel("Telemetry: waiting for data")
        self.summary_label.setWordWrap(True)
        self.summary_label.setObjectName("SectionHint")
        layout.addWidget(self.summary_label)

        if pg is None:
            self._plotting_available = False
            unavailable_label = QLabel(
                f"Charts unavailable: pyqtgraph import failed ({_pyqtgraph_import_error})"
            )
            unavailable_label.setWordWrap(True)
            layout.addWidget(unavailable_label)
            return

        self._plotting_available = True

        joint_row = QHBoxLayout()
        joint_row.addWidget(QLabel("Joint"))
        self.joint_selector = QComboBox()
        self.joint_selector.setMinimumWidth(220)
        joint_row.addWidget(self.joint_selector)
        layout.addLayout(joint_row)

        self.joint_position_plot = self._create_plot_widget("Joint Position", "position")
        self.joint_position_curve = self.joint_position_plot.plot(
            pen=pg.mkPen("#1f77b4", width=2),
            name="position",
        )
        layout.addWidget(self.joint_position_plot)

        self.joint_velocity_plot = self._create_plot_widget("Joint Velocity", "velocity")
        self.joint_velocity_curve = self.joint_velocity_plot.plot(
            pen=pg.mkPen("#d62728", width=2),
            name="velocity",
        )
        layout.addWidget(self.joint_velocity_plot)

        self.motion_plot = self._create_plot_widget("Cmd vs Odom Speed", "value")
        self.cmd_linear_curve = self.motion_plot.plot(
            pen=pg.mkPen("#2ca02c", width=2),
            name="cmd linear",
        )
        self.odom_linear_curve = self.motion_plot.plot(
            pen=pg.mkPen("#17becf", width=2),
            name="odom linear",
        )
        self.cmd_angular_curve = self.motion_plot.plot(
            pen=pg.mkPen("#9467bd", width=2, style=pg.QtCore.Qt.DashLine),
            name="cmd angular",
        )
        self.odom_angular_curve = self.motion_plot.plot(
            pen=pg.mkPen("#8c564b", width=2, style=pg.QtCore.Qt.DashLine),
            name="odom angular",
        )
        self._apply_motion_placeholder_data()
        layout.addWidget(self.motion_plot)

    def refresh(self) -> None:
        self.summary_label.setText("Telemetry:\n" + "\n".join(self._telemetry.topic_summary_lines()))
        if not self._plotting_available:
            return

        self._sync_joint_selector()
        sample_now_sec = time.monotonic()
        selected_joint = self.joint_selector.currentText().strip()
        if selected_joint:
            position_x, position_y = self._telemetry.get_joint_position_series(selected_joint, sample_now_sec)
            velocity_x, velocity_y = self._telemetry.get_joint_velocity_series(selected_joint, sample_now_sec)
        else:
            position_x, position_y = [], []
            velocity_x, velocity_y = [], []

        cmd_linear_x, cmd_linear_y = self._telemetry.get_cmd_linear_series(sample_now_sec)
        odom_linear_x, odom_linear_y = self._telemetry.get_odom_linear_series(sample_now_sec)
        cmd_angular_x, cmd_angular_y = self._telemetry.get_cmd_angular_series(sample_now_sec)
        odom_angular_x, odom_angular_y = self._telemetry.get_odom_angular_series(sample_now_sec)

        self.joint_position_curve.setData(position_x, position_y)
        self.joint_velocity_curve.setData(velocity_x, velocity_y)
        self.cmd_linear_curve.setData(*self._series_or_placeholder(cmd_linear_x, cmd_linear_y))
        self.odom_linear_curve.setData(*self._series_or_placeholder(odom_linear_x, odom_linear_y))
        self.cmd_angular_curve.setData(*self._series_or_placeholder(cmd_angular_x, cmd_angular_y))
        self.odom_angular_curve.setData(*self._series_or_placeholder(odom_angular_x, odom_angular_y))

        for plot_widget in self._plot_widgets:
            plot_widget.setXRange(-self._telemetry.history_sec, 0.0, padding=0.0)

    def _sync_joint_selector(self) -> None:
        joint_names = self._telemetry.joint_names()
        if joint_names == self._joint_names:
            return
        current_name = self.joint_selector.currentText().strip()
        self._joint_names = joint_names
        self.joint_selector.blockSignals(True)
        self.joint_selector.clear()
        self.joint_selector.addItems(joint_names)
        if current_name in joint_names:
            self.joint_selector.setCurrentText(current_name)
        self.joint_selector.blockSignals(False)

    def _create_plot_widget(self, title: str, y_label: str):
        plot_widget = pg.PlotWidget(title=title)
        plot_widget.setMinimumHeight(190)
        plot_widget.showGrid(x=True, y=True, alpha=0.18)
        plot_widget.setLabel("left", y_label)
        plot_widget.setLabel("bottom", "seconds", units="ago")
        legend = plot_widget.addLegend(offset=(12, 12))
        legend.setBrush(pg.mkBrush("#fffdfa"))
        legend.setPen(pg.mkPen("#d7cab8"))
        self._plot_widgets.append(plot_widget)
        return plot_widget

    def _series_or_placeholder(self, x_values: list[float], y_values: list[float]) -> tuple[list[float], list[float]]:
        if x_values and y_values:
            return x_values, y_values
        return self._motion_placeholder_x, self._motion_placeholder_y

    def _apply_motion_placeholder_data(self) -> None:
        self.cmd_linear_curve.setData(self._motion_placeholder_x, self._motion_placeholder_y)
        self.odom_linear_curve.setData(self._motion_placeholder_x, self._motion_placeholder_y)
        self.cmd_angular_curve.setData(self._motion_placeholder_x, self._motion_placeholder_y)
        self.odom_angular_curve.setData(self._motion_placeholder_x, self._motion_placeholder_y)
