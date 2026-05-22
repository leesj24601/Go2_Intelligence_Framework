from __future__ import annotations

import io
import os
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
import sys

sys.path.insert(0, str(ROOT / "src" / "go2_gui_controller"))

from go2_gui_controller.web_launch_manager import WebLaunchManager


class _FakeProcess:
    pid = 12345
    stdout = io.StringIO("")

    def poll(self):
        return None


class WebLaunchManagerSimulationTests(unittest.TestCase):
    def test_starts_simulation_with_lab_env_python_by_default(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            conda_prefix = Path(temp_dir) / "anaconda3" / "envs" / "lab"
            python_executable = conda_prefix / "bin" / "python"
            python_executable.parent.mkdir(parents=True)
            python_executable.write_text("")
            logs: list[str] = []
            statuses: dict[str, str] = {}

            with (
                patch.dict(
                    os.environ,
                    {
                        "GO2_PROJECT_DIR": str(project_dir),
                        "GO2_SIM_CONDA_PREFIX": str(conda_prefix),
                        "GO2_SIM_CONDA_ENV": "lab",
                    },
                    clear=False,
                ),
                patch.object(WebLaunchManager, "_spawn_output_thread"),
                patch.object(WebLaunchManager, "_spawn_simulation_monitor_thread"),
                patch("go2_gui_controller.web_launch_manager.subprocess.Popen", return_value=_FakeProcess()) as popen,
            ):
                manager = WebLaunchManager("sim", logs.append, statuses.__setitem__)

                ok, message = manager.start_simulation()

            self.assertTrue(ok)
            self.assertEqual(message, "starting Simulation")
            self.assertEqual(statuses["simulation"], "running")
            popen.assert_called_once()
            args, kwargs = popen.call_args
            self.assertEqual(
                args[0],
                [str(python_executable), str(project_dir / "scripts" / "go2_sim.py")],
            )
            self.assertEqual(kwargs["cwd"], project_dir)
            self.assertTrue(kwargs["start_new_session"])
            self.assertEqual(kwargs["env"]["CONDA_PREFIX"], str(conda_prefix))
            self.assertEqual(kwargs["env"]["CONDA_DEFAULT_ENV"], "lab")
            self.assertTrue(kwargs["env"]["PATH"].startswith(str(conda_prefix / "bin")))

    def test_falls_back_to_conda_run_when_env_python_is_unavailable(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            statuses: dict[str, str] = {}

            with (
                patch.dict(
                    os.environ,
                    {
                        "GO2_PROJECT_DIR": str(project_dir),
                        "GO2_SIM_CONDA_PREFIX": str(Path(temp_dir) / "missing-env"),
                        "GO2_SIM_CONDA_EXE": "/opt/conda/bin/conda",
                        "GO2_SIM_CONDA_ENV": "lab",
                    },
                    clear=False,
                ),
                patch.object(WebLaunchManager, "_spawn_output_thread"),
                patch.object(WebLaunchManager, "_spawn_simulation_monitor_thread"),
                patch("go2_gui_controller.web_launch_manager.subprocess.Popen", return_value=_FakeProcess()) as popen,
            ):
                manager = WebLaunchManager("sim", lambda _line: None, statuses.__setitem__)

                ok, _message = manager.start_simulation()

            self.assertTrue(ok)
            args, _kwargs = popen.call_args
            self.assertEqual(
                args[0],
                [
                    "/opt/conda/bin/conda",
                    "run",
                    "--no-capture-output",
                    "-n",
                    "lab",
                    "python",
                    str(project_dir / "scripts" / "go2_sim.py"),
                ],
            )

    def test_starts_simulation_with_terminal_compatible_environment(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            conda_prefix = Path(temp_dir) / "anaconda3" / "envs" / "lab"
            python_executable = conda_prefix / "bin" / "python"
            python_executable.parent.mkdir(parents=True)
            python_executable.write_text("")
            statuses: dict[str, str] = {}

            polluted_env = {
                "GO2_PROJECT_DIR": str(project_dir),
                "GO2_SIM_CONDA_PREFIX": str(conda_prefix),
                "PATH": "/home/cvr/anaconda3/condabin:/opt/ros/humble/bin:/usr/bin",
                "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages:/tmp/custom_python",
                "AMENT_PREFIX_PATH": "/home/cvr/Desktop/sj/go2_ws/install/go2_driver:/opt/ros/humble",
                "COLCON_PREFIX_PATH": "/home/cvr/Desktop/sj/go2_ws/install",
                "CMAKE_PREFIX_PATH": "/home/cvr/Desktop/sj/ros2_orb_slam3/install/ros2_orb_slam3",
                "ROS_PACKAGE_PATH": "/opt/ros/humble/share",
                "ROS_DISTRO": "humble",
                "ROS_VERSION": "2",
                "LD_LIBRARY_PATH": (
                    "/home/cvr/Desktop/sj/go2_ws/install/unitree_api/lib:"
                    "/opt/ros/humble/lib:"
                    "/usr/local/cuda-12.4/lib64:"
                    "/usr/local/lib"
                ),
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "ROS_DOMAIN_ID": "23",
                "DISPLAY": ":0",
            }

            with (
                patch.dict(os.environ, polluted_env, clear=True),
                patch.object(WebLaunchManager, "_spawn_output_thread"),
                patch.object(WebLaunchManager, "_spawn_simulation_monitor_thread"),
                patch("go2_gui_controller.web_launch_manager.subprocess.Popen", return_value=_FakeProcess()) as popen,
            ):
                manager = WebLaunchManager("sim", lambda _line: None, statuses.__setitem__)

                ok, _message = manager.start_simulation()

            self.assertTrue(ok)
            _args, kwargs = popen.call_args
            env = kwargs["env"]
            self.assertEqual(env["PYTHONPATH"], polluted_env["PYTHONPATH"])
            self.assertEqual(env["AMENT_PREFIX_PATH"], polluted_env["AMENT_PREFIX_PATH"])
            self.assertEqual(env["COLCON_PREFIX_PATH"], polluted_env["COLCON_PREFIX_PATH"])
            self.assertEqual(env["CMAKE_PREFIX_PATH"], polluted_env["CMAKE_PREFIX_PATH"])
            self.assertEqual(env["ROS_PACKAGE_PATH"], polluted_env["ROS_PACKAGE_PATH"])
            self.assertEqual(env["ROS_DISTRO"], "humble")
            self.assertEqual(env["ROS_VERSION"], "2")
            self.assertEqual(env["LD_LIBRARY_PATH"], polluted_env["LD_LIBRARY_PATH"])
            self.assertEqual(env["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
            self.assertEqual(env["ROS_DOMAIN_ID"], "23")
            self.assertEqual(env["DISPLAY"], ":0")
            self.assertEqual(env["CONDA_PREFIX"], str(conda_prefix))
            self.assertEqual(env["CONDA_DEFAULT_ENV"], "lab")
            self.assertTrue(env["PATH"].startswith(str(conda_prefix / "bin")))

    def test_simulation_launch_is_disabled_in_real_mode(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            statuses: dict[str, str] = {}

            with patch.dict(os.environ, {"GO2_PROJECT_DIR": str(project_dir)}, clear=False):
                manager = WebLaunchManager("real", lambda _line: None, statuses.__setitem__)

            ok, message = manager.start_simulation()

            self.assertFalse(ok)
            self.assertEqual(message, "Simulation launcher is only available in sim mode")
            self.assertEqual(statuses["simulation"], "disabled")

    def test_navigation_uses_selected_map_database(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            (project_dir / "maps" / "lab.db").write_text("")
            statuses: dict[str, str] = {}

            with (
                patch.dict(os.environ, {"GO2_PROJECT_DIR": str(project_dir)}, clear=False),
                patch.object(WebLaunchManager, "_spawn_output_thread"),
                patch.object(WebLaunchManager, "_spawn_monitor_thread"),
                patch("go2_gui_controller.web_launch_manager.subprocess.Popen", return_value=_FakeProcess()) as popen,
            ):
                manager = WebLaunchManager("sim", lambda _line: None, statuses.__setitem__)

                ok, message = manager.start("navigation", map_name="lab")

            self.assertTrue(ok)
            self.assertEqual(message, "starting Navigation stack")
            args, kwargs = popen.call_args
            self.assertEqual(
                args[0],
                [
                    "ros2",
                    "launch",
                    str(project_dir / "launch" / "go2_navigation.launch.py"),
                    f"localization_db:={project_dir / 'maps' / 'lab.db'}",
                ],
            )
            self.assertEqual(kwargs["cwd"], project_dir)
            self.assertEqual(statuses["navigation"], "running")

    def test_slam_can_start_with_named_database(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            statuses: dict[str, str] = {}

            with (
                patch.dict(os.environ, {"GO2_PROJECT_DIR": str(project_dir)}, clear=False),
                patch.object(WebLaunchManager, "_spawn_output_thread"),
                patch.object(WebLaunchManager, "_spawn_monitor_thread"),
                patch("go2_gui_controller.web_launch_manager.subprocess.Popen", return_value=_FakeProcess()) as popen,
            ):
                manager = WebLaunchManager("sim", lambda _line: None, statuses.__setitem__)

                ok, _message = manager.start("slam", map_name="new_office")

            self.assertTrue(ok)
            args, _kwargs = popen.call_args
            self.assertEqual(
                args[0],
                [
                    "ros2",
                    "launch",
                    str(project_dir / "launch" / "go2_rtabmap.launch.py"),
                    f"slam_db:={project_dir / 'maps' / 'new_office.db'}",
                ],
            )

    def test_save_map_runs_nav2_map_saver(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            calls: list[list[str]] = []

            def fake_run(command, **_kwargs):
                calls.append(command)
                return types.SimpleNamespace(returncode=0, stdout="saved", stderr="")

            with (
                patch.dict(os.environ, {"GO2_PROJECT_DIR": str(project_dir)}, clear=False),
                patch("go2_gui_controller.web_launch_manager.subprocess.run", side_effect=fake_run),
            ):
                manager = WebLaunchManager("sim", lambda _line: None, lambda _key, _status: None)

                ok, message = manager.save_map("lab")

            self.assertTrue(ok)
            self.assertEqual(message, "saved map lab")
            self.assertEqual(
                calls,
                [
                    [
                        "ros2",
                        "run",
                        "nav2_map_server",
                        "map_saver_cli",
                        "-f",
                        str(project_dir / "maps" / "lab"),
                    ]
                ],
            )

    def test_rejects_unsafe_map_names(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))

            with patch.dict(os.environ, {"GO2_PROJECT_DIR": str(project_dir)}, clear=False):
                manager = WebLaunchManager("sim", lambda _line: None, lambda _key, _status: None)

            ok, message = manager.start("slam", map_name="../bad")

            self.assertFalse(ok)
            self.assertIn("map name", message)

    def test_force_cleanup_targets_stale_launches_and_orphan_runtime_nodes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            project_dir = _write_project_fixture(Path(temp_dir))
            calls: list[list[str]] = []

            def fake_run(command, **_kwargs):
                calls.append(command)
                return types.SimpleNamespace(returncode=1, stderr="")

            with (
                patch.dict(os.environ, {"GO2_PROJECT_DIR": str(project_dir)}, clear=False),
                patch("go2_gui_controller.web_launch_manager.subprocess.run", side_effect=fake_run),
            ):
                manager = WebLaunchManager("sim", lambda _line: None, lambda _key, _status: None)

                ok, message = manager.force_cleanup_runtime()

            self.assertTrue(ok)
            self.assertEqual(message, "force cleanup sent")
            for pattern in (
                "go2_rtabmap.launch.py",
                "go2_navigation.launch.py",
                "depthimage_to_laserscan",
                "static_transform_publisher",
            ):
                self.assertIn(["pkill", "-INT", "-f", pattern], calls)


def _write_project_fixture(project_dir: Path) -> Path:
    for dirname in ("launch", "config", "scripts", "maps"):
        (project_dir / dirname).mkdir()

    for filename in (
        "go2_rtabmap.launch.py",
        "go2_navigation.launch.py",
        "go2_rtabmap_real.launch.py",
        "go2_navigation_real.launch.py",
    ):
        (project_dir / "launch" / filename).write_text("")

    (project_dir / "config" / "go2_sim.rviz").write_text("")
    (project_dir / "scripts" / "go2_sim.py").write_text("")
    return project_dir


if __name__ == "__main__":
    unittest.main()
