from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import threading
from typing import Callable


LogCallback = Callable[[str], None]
StatusCallback = Callable[[str, str], None]


@dataclass(frozen=True)
class StackSpec:
    key: str
    label: str
    launch_file: Path


class WebLaunchManager:
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
        self._lock = threading.Lock()
        self._pending_start_key: str | None = None
        self._stop_requested: dict[str, bool] = {}
        self._processes: dict[str, subprocess.Popen[str] | None] = {}
        self._states: dict[str, str] = {}
        self._rviz_process: subprocess.Popen[str] | None = None
        self._rviz_state = "stopped"
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
            self._processes[key] = None
            self._states[key] = "stopped"
            self._status_callback(key, "stopped")
        self._rviz_config_file = self._project_dir / "config" / "go2_sim.rviz"
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
        with self._lock:
            return self._states.get(key, "stopped")

    def rviz_status_text(self) -> str:
        if not self.available:
            return f"unavailable ({self._unavailable_reason})"
        with self._lock:
            return self._rviz_state

    def is_process_running(self, key: str) -> bool:
        if not self.available:
            return False
        with self._lock:
            return self._process_alive(self._processes.get(key))

    def start(self, key: str) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if key not in self._specs:
            return False, f"unknown stack: {key}"

        other_key = "navigation" if key == "slam" else "slam"
        with self._lock:
            if self._process_alive(self._processes[key]):
                return False, f"{self._specs[key].label} stack is already running"
            if self._process_alive(self._processes[other_key]):
                self._pending_start_key = key
                self._log(
                    f"[Runtime] stopping {self._specs[other_key].label} stack before starting {self._specs[key].label}"
                )
            else:
                self._pending_start_key = None

        if self._process_alive(self._processes.get(other_key)):
            self.stop(other_key)
            return True, f"stopping {self._specs[other_key].label} stack before starting {self._specs[key].label}"

        self._start_process(key)
        return True, f"starting {self._specs[key].label} stack"

    def stop(self, key: str) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if key not in self._specs:
            return False, f"unknown stack: {key}"

        with self._lock:
            process = self._processes.get(key)
            if not self._process_alive(process):
                if self._pending_start_key == key:
                    self._pending_start_key = None
                return False, f"{self._specs[key].label} stack is not running"
            self._stop_requested[key] = True
            self._states[key] = "stopping"
            self._status_callback(key, "stopping")
            assert process is not None
            self._terminate_group(process)
        threading.Timer(3.0, lambda managed_key=key: self._kill_if_needed(managed_key)).start()
        return True, f"stopping {self._specs[key].label} stack"

    def open_rviz(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason

        with self._lock:
            if self._process_alive(self._rviz_process):
                return False, "RViz is already running"
            self._rviz_stop_requested = False
            self._rviz_state = "starting"
            self._status_callback("rviz", "starting")

        process = subprocess.Popen(
            ["rviz2", "-d", str(self._rviz_config_file)],
            cwd=self._project_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        with self._lock:
            self._rviz_process = process
            self._rviz_state = "running"
            self._status_callback("rviz", "running")
        self._log("[Runtime] RViz started")
        self._spawn_output_thread(process, "RViz")
        self._spawn_rviz_monitor_thread(process)
        return True, "starting RViz"

    def close_rviz(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason

        with self._lock:
            if not self._process_alive(self._rviz_process):
                return False, "RViz is not running"
            self._rviz_stop_requested = True
            self._rviz_state = "stopping"
            self._status_callback("rviz", "stopping")
            assert self._rviz_process is not None
            self._terminate_group(self._rviz_process)
        threading.Timer(3.0, self._kill_rviz_if_needed).start()
        return True, "stopping RViz"

    def shutdown(self) -> None:
        if not self.available:
            return

        with self._lock:
            self._pending_start_key = None
            rviz_process = self._rviz_process
            processes = {key: process for key, process in self._processes.items() if process is not None}
            for key in processes:
                self._stop_requested[key] = True
            self._rviz_stop_requested = rviz_process is not None

        if rviz_process is not None and self._process_alive(rviz_process):
            self._terminate_group(rviz_process)
            try:
                rviz_process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self._kill_group(rviz_process)

        for process in processes.values():
            if not self._process_alive(process):
                continue
            self._terminate_group(process)
        for process in processes.values():
            if not self._process_alive(process):
                continue
            try:
                process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self._kill_group(process)

    def _start_process(self, key: str) -> None:
        spec = self._specs[key]
        with self._lock:
            self._stop_requested[key] = False
            self._states[key] = "starting"
            self._status_callback(key, "starting")

        process = subprocess.Popen(
            ["ros2", "launch", str(spec.launch_file)],
            cwd=self._project_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        with self._lock:
            self._processes[key] = process
            self._states[key] = "running"
            self._status_callback(key, "running")
        self._log(f"[Runtime] {spec.label} stack started")
        self._spawn_output_thread(process, spec.label)
        self._spawn_monitor_thread(key, process)

    def _spawn_output_thread(self, process: subprocess.Popen[str], label: str) -> None:
        thread = threading.Thread(target=self._drain_output, args=(process, label), daemon=True)
        thread.start()

    def _spawn_monitor_thread(self, key: str, process: subprocess.Popen[str]) -> None:
        thread = threading.Thread(target=self._monitor_process, args=(key, process), daemon=True)
        thread.start()

    def _spawn_rviz_monitor_thread(self, process: subprocess.Popen[str]) -> None:
        thread = threading.Thread(target=self._monitor_rviz_process, args=(process,), daemon=True)
        thread.start()

    def _drain_output(self, process: subprocess.Popen[str], label: str) -> None:
        if process.stdout is None:
            return
        try:
            for line in process.stdout:
                stripped = line.strip()
                if stripped:
                    self._log(f"[{label}] {stripped}")
        finally:
            try:
                process.stdout.close()
            except Exception:
                pass

    def _monitor_process(self, key: str, process: subprocess.Popen[str]) -> None:
        exit_code = process.wait()
        pending_key: str | None = None
        with self._lock:
            if self._processes.get(key) is process:
                self._processes[key] = None
            was_stop_requested = self._stop_requested.get(key, False)
            self._stop_requested[key] = False

            if was_stop_requested:
                state = "stopped"
                message = f"[Runtime] {self._specs[key].label} stack stopped"
            elif exit_code < 0:
                state = "crashed"
                message = f"[Runtime] {self._specs[key].label} stack crashed"
            else:
                state = f"exited ({exit_code})"
                message = f"[Runtime] {self._specs[key].label} stack exited with code {exit_code}"

            self._states[key] = state
            self._status_callback(key, state)
            self._log(message)

            if self._pending_start_key and self._all_processes_stopped_locked():
                pending_key = self._pending_start_key
                self._pending_start_key = None

        if pending_key is not None:
            self._start_process(pending_key)

    def _monitor_rviz_process(self, process: subprocess.Popen[str]) -> None:
        exit_code = process.wait()
        with self._lock:
            if self._rviz_process is process:
                self._rviz_process = None
            was_stop_requested = self._rviz_stop_requested
            self._rviz_stop_requested = False

            if was_stop_requested:
                state = "stopped"
                message = "[Runtime] RViz stopped"
            elif exit_code < 0:
                state = "crashed"
                message = "[Runtime] RViz crashed"
            else:
                state = f"exited ({exit_code})"
                message = f"[Runtime] RViz exited with code {exit_code}"

            self._rviz_state = state
            self._status_callback("rviz", state)
            self._log(message)

    def _kill_if_needed(self, key: str) -> None:
        with self._lock:
            process = self._processes.get(key)
        if not self._process_alive(process):
            return
        self._log(f"[Runtime] force-killing {self._specs[key].label} stack")
        assert process is not None
        self._kill_group(process)

    def _kill_rviz_if_needed(self) -> None:
        with self._lock:
            process = self._rviz_process
        if not self._process_alive(process):
            return
        self._log("[Runtime] force-killing RViz")
        assert process is not None
        self._kill_group(process)

    @staticmethod
    def _terminate_group(process: subprocess.Popen[str]) -> None:
        """프로세스 그룹 전체에 SIGTERM — ros2 launch 자식들도 함께 종료."""
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    @staticmethod
    def _kill_group(process: subprocess.Popen[str]) -> None:
        """프로세스 그룹 전체에 SIGKILL — orphan 방지."""
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass

    def _process_alive(self, process: subprocess.Popen[str] | None) -> bool:
        return process is not None and process.poll() is None

    def _all_processes_stopped_locked(self) -> bool:
        return all(not self._process_alive(process) for process in self._processes.values())

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
