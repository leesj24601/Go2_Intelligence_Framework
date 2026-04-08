---
tags: [project, dependencies, setup]
status: active
project: go2_intelligence_framework
type: reference
created: 2026-02-24
---

# Project Dependencies

## Manual Install

- Ubuntu `22.04`
- ROS 2 `Humble`
- NVIDIA Isaac Sim `5.1.0`
- NVIDIA Isaac Lab `2.3.0`
- `go2_description` in a separate ROS workspace

## Pip Install

File: `requirements.txt`

- `faster-whisper`
- `sounddevice`
- `PyYAML`

## ROS / System Install

Installed via:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### From `go2_gui_controller/package.xml`

- `action_msgs`
- `geometry_msgs`
- `nav_msgs`
- `nav2_msgs`
- `nav2_simple_commander`
- `portaudio19-dev`
- `python3-numpy`
- `python3-pyqtgraph`
- `python_qt_binding`
- `rclpy`
- `sensor_msgs`
- `tf2_ros`
- `ament_index_python`

### From `go2_project_dependencies/package.xml`

- `joint_state_publisher`
- `nav2_bringup`
- `depthimage_to_laserscan`
- `robot_state_publisher`
- `rtabmap_ros`
- `rviz2`
- `xacro`

## Recommended Install Split

- `pip`: `faster-whisper`, `sounddevice`
- `rosdep` / system: ROS packages, `python3-numpy`, `python3-pyqtgraph`, `portaudio19-dev`
- manual: Isaac Sim, Isaac Lab, `go2_description`

## Local Check on 2026-03-12

### Installed / Available

- `python3-numpy` `1.21.5-1ubuntu22.04.1`
- `python3-pyqtgraph` installed manually via `sudo apt install python3-pyqtgraph`
- `python3-pyqt5` `5.15.6+dfsg-1ubuntu3`
- `ros-humble-python-qt-binding` `1.1.3-1jammy.20251008.150450`
- `ros-humble-nav2-bringup` `1.1.20-1jammy.20251228.133403`
- `ros-humble-rtabmap-ros` `0.22.1-1jammy.20251228.133627`
- `ros-humble-rviz2` `11.2.25-1jammy.20251228.130645`
- `ros-humble-rqt-plot` `1.1.5-1jammy.20251222.193959`
- `ros-humble-foxglove-bridge` `3.2.2-1jammy.20251228.114733`

### Not Found

- `plotjuggler`
- `plotjuggler_ros`

## Note

- `numpy` is intentionally not in `requirements.txt`.
- Use system `python3-numpy` instead of pip `numpy` in this project.
- `faster-whisper` and `sounddevice` stay in `requirements.txt` because no rosdep rule was found for them.
