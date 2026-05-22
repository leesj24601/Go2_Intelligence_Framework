from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import threading
import time
from typing import Callable


LogCallback = Callable[[str], None]
StatusCallback = Callable[[str, str], None]


@dataclass(frozen=True)
class StackSpec:
    key: str
    label: str
    launch_file: Path


class WebLaunchManager:
    STACK_TERMINATION_GRACE_SEC = 15.0
    SIMULATION_TERMINATION_GRACE_SEC = 30.0
    RVIZ_TERMINATION_GRACE_SEC = 3.0
    FORCE_CLEANUP_SIGNAL_GRACE_SEC = 0.2
    FORCE_CLEANUP_SIGNALS = ("INT", "TERM", "KILL")
    FORCE_CLEANUP_PATTERNS = (
        "go2_sim.py",
        "go2_rtabmap.launch.py",
        "go2_navigation.launch.py",
        "go2_rtabmap_real.launch.py",
        "go2_navigation_real.launch.py",
        "rtabmap_slam",
        "depthimage_to_laserscan",
        "static_transform_publisher",
        "nav2_",
        "robot_state_publisher",
        "rviz2",
    )
    MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

    REQUIRED_LAUNCH_FILES = (
        ("launch", "go2_rtabmap.launch.py"),
        ("launch", "go2_navigation.launch.py"),
        ("launch", "go2_rtabmap_real.launch.py"),
        ("launch", "go2_navigation_real.launch.py"),
        ("config", "go2_sim.rviz"),
        ("scripts", "go2_sim.py"),
    )

    def __init__(self, runtime_mode_key: str, log_callback: LogCallback, status_callback: StatusCallback):
        self._runtime_mode_key = runtime_mode_key
        self._log = log_callback
        self._status_callback = status_callback
        self._project_dir, self._unavailable_reason = self._resolve_project_dir()
        self._lock = threading.Lock()
        self._pending_start_key: str | None = None
        self._pending_start_map_name: str | None = None
        self._stop_requested: dict[str, bool] = {}
        self._processes: dict[str, subprocess.Popen[str] | None] = {}
        self._states: dict[str, str] = {}
        self._simulation_process: subprocess.Popen[str] | None = None
        self._simulation_state = "disabled" if runtime_mode_key != "sim" else "stopped"
        self._simulation_stop_requested = False
        self._rviz_process: subprocess.Popen[str] | None = None
        self._rviz_state = "stopped"
        self._rviz_stop_requested = False
        self._selected_map_name = "rtabmap_office" if runtime_mode_key == "sim" else "rtabmap_real"

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
        self._simulation_script = self._project_dir / "scripts" / "go2_sim.py"
        self._status_callback("simulation", self._simulation_state)
        self._rviz_config_file = self._project_dir / "config" / "go2_sim.rviz"
        self._maps_dir = self._project_dir / "maps"
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

    def simulation_status_text(self) -> str:
        if not self.available:
            return f"unavailable ({self._unavailable_reason})"
        with self._lock:
            return self._simulation_state

    def is_process_running(self, key: str) -> bool:
        if not self.available:
            return False
        with self._lock:
            return self._process_alive(self._processes.get(key))

    def is_simulation_running(self) -> bool:
        if not self.available:
            return False
        with self._lock:
            return self._process_alive(self._simulation_process)

    def start(self, key: str, map_name: str | None = None) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if key not in self._specs:
            return False, f"unknown stack: {key}"
        try:
            launch_map_name = self._resolve_launch_map_name(key, map_name)
        except ValueError as exc:
            return False, str(exc)

        other_key = "navigation" if key == "slam" else "slam"
        with self._lock:
            if self._process_alive(self._processes[key]):
                return False, f"{self._specs[key].label} stack is already running"
            if self._process_alive(self._processes[other_key]):
                self._pending_start_key = key
                self._pending_start_map_name = launch_map_name
                self._log(
                    f"[Runtime] stopping {self._specs[other_key].label} stack before starting {self._specs[key].label}"
                )
            else:
                self._pending_start_key = None
                self._pending_start_map_name = None

        if self._process_alive(self._processes.get(other_key)):
            self.stop(other_key)
            return True, f"stopping {self._specs[other_key].label} stack before starting {self._specs[key].label}"

        self._start_process(key, launch_map_name)
        return True, f"starting {self._specs[key].label} stack"

    def list_maps(self) -> list[dict[str, object]]:
        if not self.available:
            return []
        maps_dir = self._maps_dir
        if not maps_dir.is_dir():
            return []
        names = {
            path.stem
            for path in maps_dir.iterdir()
            if path.is_file() and path.suffix in (".db", ".yaml", ".pgm")
        }
        maps = []
        for name in sorted(names):
            maps.append(
                {
                    "name": name,
                    "db": (maps_dir / f"{name}.db").is_file(),
                    "yaml": (maps_dir / f"{name}.yaml").is_file(),
                    "pgm": (maps_dir / f"{name}.pgm").is_file(),
                    "selected": name == self.selected_map_name,
                }
            )
        return maps

    @property
    def selected_map_name(self) -> str:
        return self._selected_map_name

    def select_map(self, map_name: str) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        try:
            clean_name = self._sanitize_map_name(map_name)
        except ValueError as exc:
            return False, str(exc)
        db_path = self._map_db_path(clean_name)
        if not db_path.is_file():
            return False, f"map db not found: {db_path}"
        self._selected_map_name = clean_name
        self._log(f"[Runtime] selected map: {clean_name}")
        return True, f"selected map {clean_name}"

    def save_map(self, map_name: str) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        try:
            clean_name = self._sanitize_map_name(map_name)
        except ValueError as exc:
            return False, str(exc)
        self._maps_dir.mkdir(parents=True, exist_ok=True)
        output_prefix = self._maps_dir / clean_name
        command = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(output_prefix),
        ]
        try:
            result = subprocess.run(
                command,
                cwd=self._project_dir,
                text=True,
                capture_output=True,
                timeout=30.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._log(f"[Runtime] map save failed: {exc}")
            return False, f"map save failed: {exc}"
        output = "\n".join(part for part in (result.stdout.strip(), result.stderr.strip()) if part)
        if result.returncode != 0:
            if output:
                self._log(f"[Runtime] map save failed: {output}")
            return False, output or f"map save failed with code {result.returncode}"
        if self._map_db_path(clean_name).is_file():
            self._selected_map_name = clean_name
        self._log(f"[Runtime] map saved: {output_prefix}.yaml/.pgm")
        return True, f"saved map {clean_name}"

    def start_simulation(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if self._runtime_mode_key != "sim":
            return False, "Simulation launcher is only available in sim mode"

        with self._lock:
            if self._process_alive(self._simulation_process):
                return False, "Simulation is already running"
            self._simulation_stop_requested = False
            self._simulation_state = "starting"
            self._status_callback("simulation", "starting")

        command = self._simulation_command()
        try:
            process = subprocess.Popen(
                command,
                cwd=self._project_dir,
                env=self._simulation_environment(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except OSError as exc:
            with self._lock:
                self._simulation_state = "stopped"
                self._status_callback("simulation", "stopped")
            self._log(f"[Runtime] Simulation failed to start: {exc}")
            return False, f"Simulation failed to start: {exc}"

        with self._lock:
            self._simulation_process = process
            self._simulation_state = "running"
            self._status_callback("simulation", "running")
        self._log(f"[Runtime] Simulation started ({' '.join(command)})")
        self._spawn_output_thread(process, "Simulation")
        self._spawn_simulation_monitor_thread(process)
        return True, "starting Simulation"

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
                    self._pending_start_map_name = None
                return False, f"{self._specs[key].label} stack is not running"
            self._stop_requested[key] = True
            self._states[key] = "stopping"
            self._status_callback(key, "stopping")
            assert process is not None
            self._interrupt_group(process)
        self._log(
            f"[Runtime] waiting up to {self.STACK_TERMINATION_GRACE_SEC:.0f}s for {self._specs[key].label} stack shutdown"
        )
        threading.Timer(
            self.STACK_TERMINATION_GRACE_SEC,
            lambda managed_key=key: self._kill_if_needed(managed_key),
        ).start()
        return True, f"stopping {self._specs[key].label} stack"

    def stop_simulation(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason
        if self._runtime_mode_key != "sim":
            return False, "Simulation launcher is only available in sim mode"

        with self._lock:
            if not self._process_alive(self._simulation_process):
                return False, "Simulation is not running"
            self._simulation_stop_requested = True
            self._simulation_state = "stopping"
            self._status_callback("simulation", "stopping")
            assert self._simulation_process is not None
            self._interrupt_group(self._simulation_process)
        self._log(
            f"[Runtime] waiting up to {self.SIMULATION_TERMINATION_GRACE_SEC:.0f}s for Simulation shutdown"
        )
        threading.Timer(self.SIMULATION_TERMINATION_GRACE_SEC, self._kill_simulation_if_needed).start()
        return True, "stopping Simulation"

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
        threading.Timer(self.RVIZ_TERMINATION_GRACE_SEC, self._kill_rviz_if_needed).start()
        return True, "stopping RViz"

    def force_cleanup_runtime(self) -> tuple[bool, str]:
        if not self.available:
            return False, self._unavailable_reason

        with self._lock:
            self._pending_start_key = None
            self._pending_start_map_name = None
            known_processes = [process for process in self._processes.values() if self._process_alive(process)]
            simulation_process = self._simulation_process if self._process_alive(self._simulation_process) else None
            rviz_process = self._rviz_process if self._process_alive(self._rviz_process) else None
            for key in self._specs:
                self._stop_requested[key] = True
                self._states[key] = "stopped"
                self._status_callback(key, "stopped")
            self._simulation_stop_requested = simulation_process is not None
            self._simulation_state = "stopped" if self._runtime_mode_key == "sim" else "disabled"
            self._status_callback("simulation", self._simulation_state)
            self._rviz_stop_requested = True
            self._rviz_state = "stopped"
            self._status_callback("rviz", "stopped")

        for process in known_processes:
            assert process is not None
            self._interrupt_group(process)
        if simulation_process is not None:
            self._interrupt_group(simulation_process)
        if rviz_process is not None:
            self._terminate_group(rviz_process)

        killed_patterns: list[str] = []
        missing_patterns: list[str] = []
        failed_patterns: list[str] = []
        for pattern in self.FORCE_CLEANUP_PATTERNS:
            status, error = self._force_cleanup_pattern(pattern)
            if status == "killed":
                killed_patterns.append(pattern)
            elif status == "missing":
                missing_patterns.append(pattern)
            else:
                failed_patterns.append(pattern)
                self._log(f"[Runtime] force cleanup failed for {pattern}: {error or 'unknown error'}")

        self._log(
            "[Runtime] force cleanup requested "
            f"(killed={killed_patterns or '-'}, already_stopped={missing_patterns or '-'})"
        )
        if failed_patterns:
            return False, f"force cleanup failed for: {', '.join(failed_patterns)}"
        return True, "force cleanup sent"

    def _force_cleanup_pattern(self, pattern: str) -> tuple[str, str]:
        signaled = False
        for signal_name in self.FORCE_CLEANUP_SIGNALS:
            result = subprocess.run(
                ["pkill", f"-{signal_name}", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
            if result.returncode == 0:
                signaled = True
                if signal_name != self.FORCE_CLEANUP_SIGNALS[-1]:
                    time.sleep(self.FORCE_CLEANUP_SIGNAL_GRACE_SEC)
                continue
            if result.returncode == 1:
                return ("killed" if signaled else "missing"), ""
            return "failed", result.stderr.strip() or str(result.returncode)
        return ("killed" if signaled else "missing"), ""

    def shutdown(self) -> None:
        if not self.available:
            return

        with self._lock:
            self._pending_start_key = None
            simulation_process = self._simulation_process
            rviz_process = self._rviz_process
            processes = {key: process for key, process in self._processes.items() if process is not None}
            for key in processes:
                self._stop_requested[key] = True
            self._simulation_stop_requested = simulation_process is not None
            self._rviz_stop_requested = rviz_process is not None

        if simulation_process is not None and self._process_alive(simulation_process):
            self._interrupt_group(simulation_process)
            try:
                simulation_process.wait(timeout=self.SIMULATION_TERMINATION_GRACE_SEC)
            except subprocess.TimeoutExpired:
                self._kill_group(simulation_process)

        if rviz_process is not None and self._process_alive(rviz_process):
            self._terminate_group(rviz_process)
            try:
                rviz_process.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self._kill_group(rviz_process)

        for process in processes.values():
            if not self._process_alive(process):
                continue
            self._interrupt_group(process)
        for process in processes.values():
            if not self._process_alive(process):
                continue
            try:
                process.wait(timeout=self.STACK_TERMINATION_GRACE_SEC)
            except subprocess.TimeoutExpired:
                self._kill_group(process)

    def _start_process(self, key: str, map_name: str | None = None) -> None:
        spec = self._specs[key]
        with self._lock:
            self._stop_requested[key] = False
            self._states[key] = "starting"
            self._status_callback(key, "starting")

        command = ["ros2", "launch", str(spec.launch_file)]
        command.extend(self._launch_arguments(key, map_name))
        process = subprocess.Popen(
            command,
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
        map_suffix = f" map={map_name}" if map_name else ""
        self._log(f"[Runtime] {spec.label} stack started{map_suffix}")
        self._spawn_output_thread(process, spec.label)
        self._spawn_monitor_thread(key, process)

    def _resolve_launch_map_name(self, key: str, map_name: str | None) -> str | None:
        if key == "slam":
            if not map_name or not str(map_name).strip():
                return None
            return self._sanitize_map_name(map_name)
        if key == "navigation":
            clean_name = self._sanitize_map_name(map_name or self._selected_map_name)
            db_path = self._map_db_path(clean_name)
            if not db_path.is_file():
                raise ValueError(f"map db not found: {db_path}")
            self._selected_map_name = clean_name
            return clean_name
        return None

    def _launch_arguments(self, key: str, map_name: str | None) -> list[str]:
        if not map_name:
            return []
        if key == "slam":
            return [f"slam_db:={self._map_db_path(map_name)}"]
        if key == "navigation":
            return [f"localization_db:={self._map_db_path(map_name)}"]
        return []

    def _sanitize_map_name(self, map_name: str) -> str:
        clean_name = str(map_name or "").strip()
        if not self.MAP_NAME_PATTERN.fullmatch(clean_name):
            raise ValueError("map name must use letters, numbers, dot, dash, or underscore")
        if clean_name in (".", "..") or "/" in clean_name or "\\" in clean_name:
            raise ValueError("invalid map name")
        return clean_name

    def _map_db_path(self, map_name: str) -> Path:
        return self._maps_dir / f"{map_name}.db"

    def _simulation_command(self) -> list[str]:
        python_executable = os.environ.get("GO2_SIM_PYTHON", "").strip()
        if python_executable:
            return [python_executable, str(self._simulation_script)]

        conda_env = os.environ.get("GO2_SIM_CONDA_ENV", "lab").strip() or "lab"
        conda_prefix = self._simulation_conda_prefix(conda_env)
        env_python = conda_prefix / "bin" / "python"
        if env_python.is_file():
            return [str(env_python), str(self._simulation_script)]

        conda_executable = os.environ.get("GO2_SIM_CONDA_EXE", "").strip()
        if not conda_executable:
            default_conda = Path.home() / "anaconda3" / "bin" / "conda"
            conda_executable = str(default_conda) if default_conda.is_file() else shutil.which("conda") or "conda"

        return [
            conda_executable,
            "run",
            "--no-capture-output",
            "-n",
            conda_env,
            "python",
            str(self._simulation_script),
        ]

    def _simulation_environment(self) -> dict[str, str]:
        env = dict(os.environ)
        conda_env = os.environ.get("GO2_SIM_CONDA_ENV", "lab").strip() or "lab"
        conda_prefix = self._simulation_conda_prefix(conda_env)
        conda_bin = str(conda_prefix / "bin")
        path_parts = [part for part in env.get("PATH", "").split(os.pathsep) if part]
        if conda_bin not in path_parts:
            path_parts.insert(0, conda_bin)
        env["PATH"] = os.pathsep.join(path_parts)
        env["CONDA_PREFIX"] = str(conda_prefix)
        env["CONDA_DEFAULT_ENV"] = conda_env
        return env

    def _simulation_conda_prefix(self, conda_env: str) -> Path:
        configured_prefix = os.environ.get("GO2_SIM_CONDA_PREFIX", "").strip()
        if configured_prefix:
            return Path(configured_prefix)
        return Path.home() / "anaconda3" / "envs" / conda_env

    def _spawn_output_thread(self, process: subprocess.Popen[str], label: str) -> None:
        thread = threading.Thread(target=self._drain_output, args=(process, label), daemon=True)
        thread.start()

    def _spawn_monitor_thread(self, key: str, process: subprocess.Popen[str]) -> None:
        thread = threading.Thread(target=self._monitor_process, args=(key, process), daemon=True)
        thread.start()

    def _spawn_simulation_monitor_thread(self, process: subprocess.Popen[str]) -> None:
        thread = threading.Thread(target=self._monitor_simulation_process, args=(process,), daemon=True)
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
        pending_map_name: str | None = None
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
                pending_map_name = self._pending_start_map_name
                self._pending_start_key = None
                self._pending_start_map_name = None

        if pending_key is not None:
            self._start_process(pending_key, pending_map_name)

    def _monitor_simulation_process(self, process: subprocess.Popen[str]) -> None:
        exit_code = process.wait()
        with self._lock:
            if self._simulation_process is process:
                self._simulation_process = None
            was_stop_requested = self._simulation_stop_requested
            self._simulation_stop_requested = False

            if self._runtime_mode_key != "sim":
                state = "disabled"
                message = "[Runtime] Simulation launcher disabled"
            elif was_stop_requested:
                state = "stopped"
                message = "[Runtime] Simulation stopped"
            elif exit_code < 0:
                state = "crashed"
                message = "[Runtime] Simulation crashed"
            else:
                state = f"exited ({exit_code})"
                message = f"[Runtime] Simulation exited with code {exit_code}"

            self._simulation_state = state
            self._status_callback("simulation", state)
            self._log(message)

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

    def _kill_simulation_if_needed(self) -> None:
        with self._lock:
            process = self._simulation_process
        if not self._process_alive(process):
            return
        self._log("[Runtime] force-killing Simulation")
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
    def _interrupt_group(process: subprocess.Popen[str]) -> None:
        """프로세스 그룹 전체에 SIGINT — 터미널 Ctrl+C와 같은 종료 경로."""
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass

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
