#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u

if [ -f "${PROJECT_DIR}/install_merge/setup.bash" ]; then
  set +u
  source "${PROJECT_DIR}/install_merge/setup.bash"
  set -u
  export AMENT_PREFIX_PATH="${PROJECT_DIR}/install_merge:${AMENT_PREFIX_PATH:-}"
  exec "${PROJECT_DIR}/install_merge/bin/gui_controller"
fi

if [ -f "${PROJECT_DIR}/install/go2_gui_controller/share/go2_gui_controller/package.bash" ]; then
  set +u
  source "${PROJECT_DIR}/install/go2_gui_controller/share/go2_gui_controller/package.bash"
  set -u
  export AMENT_PREFIX_PATH="${PROJECT_DIR}/install/go2_gui_controller:${AMENT_PREFIX_PATH:-}"
  exec "${PROJECT_DIR}/install/go2_gui_controller/bin/gui_controller"
fi

echo "GUI controller is not built yet. Run colcon build for go2_gui_controller first." >&2
exit 1
