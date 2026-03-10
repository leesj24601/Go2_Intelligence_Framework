from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .gui_app import run_app


def main() -> None:
    share_dir = Path(get_package_share_directory("go2_gui_controller"))
    waypoint_file = share_dir / "config" / "waypoints.yaml"
    run_app(waypoint_file)
