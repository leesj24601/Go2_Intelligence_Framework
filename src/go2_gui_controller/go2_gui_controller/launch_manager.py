from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Callable

try:
    from python_qt_binding.QtCore import QProcess, QTimer
except ImportError:
    from PyQt5.QtCore import QProcess, QTimer


LogCallback = Callable[[str], None]
StatusCallback = Callable[[str, str], None]


@dataclass(frozen=True)
class StackSpec:
    key: str
    label: str
    launch_file: Path


class LaunchManager:
    REQUIRED_LAUNCH_FILES = (
        ("launch", "go2_rtabmap.launch.py"),
        ("launch", "go2_navigation.launch.py"),
        ("launch", "go2_rtabmap_real.launch.py"),
        ("launch", "go2_navigation_real.launch.py"),
        ("config", "go2_sim.rviz"),
    )

    def __init__(self, runtime_mode_key: str, log_callback: LogCallback, status_callback: StatusCallback):
        self._runtime_mode_key = runtime_mode_key
        self._log = log_callback
        self._status_callback = status_callback
        self._project_dir, self._unavailable_reason = self._resolve_project_dir()
        self._pending_start_key: str | None = None
        self._stop_requested: dict[str, bool] = {}
        self._processes: dict[str, QProcess] = {}
        self._rviz_stop_requested = False

        if self._project_dir is None:
            return

        launch_suffix = "" if runtime_mode_key == "sim" else "_real"
        self._specs = {
            "slam": StackSpec(
                key="slam",
                label="SLAM",
                launch_file=self._project_dir / "launch" / f"go2_rtabmap{launch_suffix}.launch.py",
            ),
            "navigation": StackSpec(
                key="navigation",
                label="Navigation",
                launch_file=self._project_dir / "launch" / f"go2_navigation{launch_suffix}.launch.py",
            ),
        }
        for key in self._specs:
            self._stop_requested[key] = False
            self._processes[key] = self._build_process(key)
            self._status_callback(key, "stopped")
        self._rviz_config_file = self._project_dir / "config" / "go2_sim.rviz"
        self._rviz_process = self._build_rviz_process()
        self._status_callback("rviz", "stopped")

    @property
    def available(self) -> bool:
        return self._project_dir is not None

    @property
    def unavailable_reason(self) -> str:
        return self._unavailable_reason

    def status_text(self, key: str) -> str:
        if not self.available:
            return f"unavailable ({self._unavailable_reason})"
        process = self._processes[key]
        if process.state() == QProcess.Running:
            return "running"
        if process.state() == QProcess.Starting:
            return "starting"
        if self._stop_requested.get(key):
            return "stopping"
        return "stopped"

    def is_process_running(self, key: str) -> bool:
        if not self.available:
            return False
        return self._processes[key].state() in (QProcess.Starting, QProcess.Running)

    def start(self, key: str) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason

        other_key = "navigation" if key == "slam" else "slam"
        if self._processes[key].state() != QProcess.NotRunning:
            return False, f"{self._specs[key].label} stack is already running"
        if self._processes[other_key].state() != QProcess.NotRunning:
            self._pending_start_key = key
            self._log(
                f"[Runtime] stopping {self._specs[other_key].label} stack before starting {self._specs[key].label}"
            )
            self.stop(other_key)
            return True, f"stopping {self._specs[other_key].label} stack before starting {self._specs[key].label}"

        self._pending_start_key = None
        self._start_process(key)
        return True, f"starting {self._specs[key].label} stack"

    def stop(self, key: str) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        process = self._processes[key]
        if process.state() == QProcess.NotRunning:
            if self._pending_start_key == key:
                self._pending_start_key = None
            return False, f"{self._specs[key].label} stack is not running"
        self._stop_requested[key] = True
        self._status_callback(key, "stopping")
        process.terminate()
        QTimer.singleShot(3000, lambda managed_key=key: self._kill_if_needed(managed_key))
        return True, f"stopping {self._specs[key].label} stack"

    def rviz_status_text(self) -> str:
        if not self.available:
            return f"unavailable ({self._unavailable_reason})"
        if self._rviz_process.state() == QProcess.Running:
            return "running"
        if self._rviz_process.state() == QProcess.Starting:
            return "starting"
        if self._rviz_stop_requested:
            return "stopping"
        return "stopped"

    def open_rviz(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if self._rviz_process.state() != QProcess.NotRunning:
            return False, "RViz is already running"
        self._rviz_stop_requested = False
        self._rviz_process.setProgram("rviz2")
        self._rviz_process.setArguments(["-d", str(self._rviz_config_file)])
        self._rviz_process.setWorkingDirectory(str(self._project_dir))
        self._status_callback("rviz", "starting")
        self._rviz_process.start()
        return True, "starting RViz"

    def close_rviz(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if self._rviz_process.state() == QProcess.NotRunning:
            return False, "RViz is not running"
        self._rviz_stop_requested = True
        self._status_callback("rviz", "stopping")
        self._rviz_process.terminate()
        QTimer.singleShot(3000, self._kill_rviz_if_needed)
        return True, "stopping RViz"

    def shutdown(self) -> None:
        if not self.available:
            return
        self._pending_start_key = None
        if self._rviz_process.state() != QProcess.NotRunning:
            self._rviz_stop_requested = True
            self._rviz_process.terminate()
        for key in self._specs:
            process = self._processes[key]
            if process.state() == QProcess.NotRunning:
                continue
            self._stop_requested[key] = True
            process.terminate()
        if self._rviz_process.state() != QProcess.NotRunning:
            self._rviz_process.waitForFinished(1500)
            if self._rviz_process.state() != QProcess.NotRunning:
                self._rviz_process.kill()
                self._rviz_process.waitForFinished(1000)
        for key in self._specs:
            process = self._processes[key]
            if process.state() == QProcess.NotRunning:
                continue
            process.waitForFinished(1500)
            if process.state() != QProcess.NotRunning:
                process.kill()
                process.waitForFinished(1000)

    def _start_process(self, key: str) -> None:
        process = self._processes[key]
        spec = self._specs[key]
        self._stop_requested[key] = False
        process.setProgram("ros2")
        process.setArguments(["launch", str(spec.launch_file)])
        process.setWorkingDirectory(str(self._project_dir))
        self._status_callback(key, "starting")
        process.start()

    def _build_process(self, key: str) -> QProcess:
        process = QProcess()
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda managed_key=key: self._drain_output(managed_key))
        process.readyReadStandardError.connect(lambda managed_key=key: self._drain_output(managed_key))
        process.started.connect(lambda managed_key=key: self._handle_started(managed_key))
        process.finished.connect(
            lambda exit_code, exit_status, managed_key=key: self._handle_finished(
                managed_key, exit_code, exit_status
            )
        )
        process.errorOccurred.connect(lambda error, managed_key=key: self._handle_error(managed_key, error))
        return process

    def _build_rviz_process(self) -> QProcess:
        process = QProcess()
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self._drain_rviz_output)
        process.readyReadStandardError.connect(self._drain_rviz_output)
        process.started.connect(self._handle_rviz_started)
        process.finished.connect(self._handle_rviz_finished)
        process.errorOccurred.connect(self._handle_rviz_error)
        return process

    def _drain_output(self, key: str) -> None:
        process = self._processes[key]
        text = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if not text:
            text = bytes(process.readAllStandardError()).decode("utf-8", errors="replace")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped:
                self._log(f"[{self._specs[key].label}] {stripped}")

    def _drain_rviz_output(self) -> None:
        text = bytes(self._rviz_process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if not text:
            text = bytes(self._rviz_process.readAllStandardError()).decode("utf-8", errors="replace")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped:
                self._log(f"[RViz] {stripped}")

    def _handle_started(self, key: str) -> None:
        self._status_callback(key, "running")
        self._log(f"[Runtime] {self._specs[key].label} stack started")

    def _handle_finished(self, key: str, exit_code: int, _exit_status) -> None:
        was_stop_requested = self._stop_requested.get(key, False)
        self._stop_requested[key] = False
        status_text = "stopped" if was_stop_requested else f"exited ({exit_code})"
        self._status_callback(key, status_text)
        if was_stop_requested:
            self._log(f"[Runtime] {self._specs[key].label} stack stopped")
        else:
            self._log(f"[Runtime] {self._specs[key].label} stack exited with code {exit_code}")

        pending_key = self._pending_start_key
        if pending_key and self._all_processes_stopped():
            self._pending_start_key = None
            self._start_process(pending_key)

    def _handle_error(self, key: str, error) -> None:
        if error == QProcess.Crashed:
            self._status_callback(key, "crashed")
            self._log(f"[Runtime] {self._specs[key].label} stack crashed")

    def _handle_rviz_started(self) -> None:
        self._status_callback("rviz", "running")
        self._log("[Runtime] RViz started")

    def _handle_rviz_finished(self, exit_code: int, _exit_status) -> None:
        was_stop_requested = self._rviz_stop_requested
        self._rviz_stop_requested = False
        status_text = "stopped" if was_stop_requested else f"exited ({exit_code})"
        self._status_callback("rviz", status_text)
        if was_stop_requested:
            self._log("[Runtime] RViz stopped")
        else:
            self._log(f"[Runtime] RViz exited with code {exit_code}")

    def _handle_rviz_error(self, error) -> None:
        if error == QProcess.Crashed:
            self._status_callback("rviz", "crashed")
            self._log("[Runtime] RViz crashed")

    def _kill_if_needed(self, key: str) -> None:
        process = self._processes[key]
        if process.state() == QProcess.NotRunning:
            return
        self._log(f"[Runtime] force-killing {self._specs[key].label} stack")
        process.kill()

    def _kill_rviz_if_needed(self) -> None:
        if self._rviz_process.state() == QProcess.NotRunning:
            return
        self._log("[Runtime] force-killing RViz")
        self._rviz_process.kill()

    def _all_processes_stopped(self) -> bool:
        return all(process.state() == QProcess.NotRunning for process in self._processes.values())

    def _resolve_project_dir(self) -> tuple[Path | None, str]:
        candidates = []

        env_project_dir = os.environ.get("GO2_PROJECT_DIR")
        if env_project_dir:
            candidates.append(Path(env_project_dir).expanduser())

        candidates.append(Path.cwd())

        try:
            candidates.append(Path(__file__).resolve().parents[3])
        except IndexError:
            pass

        for candidate in candidates:
            candidate = candidate.resolve()
            if self._is_project_dir(candidate):
                return candidate, ""
        return None, "project directory not found; set GO2_PROJECT_DIR before launching the GUI"

    def _is_project_dir(self, candidate: Path) -> bool:
        return all((candidate / Path(*relative_path)).is_file() for relative_path in self.REQUIRED_LAUNCH_FILES)
