#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export GO2_PROJECT_DIR="${PROJECT_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

if [ -f "${PROJECT_DIR}/install/setup.bash" ]; then
  set +u
  source "${PROJECT_DIR}/install/setup.bash"
  set -u
  exec ros2 run go2_gui_controller gui_controller
fi

if [ -f "${PROJECT_DIR}/install_merge/setup.bash" ]; then
  set +u
  source "${PROJECT_DIR}/install_merge/setup.bash"
  set -u
  exec ros2 run go2_gui_controller gui_controller
fi

echo "GUI controller is not built yet. Build it from ${PROJECT_DIR} first:" >&2
echo "  cd ${PROJECT_DIR}" >&2
echo "  colcon build --symlink-install --packages-select go2_gui_controller" >&2
exit 1
