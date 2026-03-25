<div align="center">
  <img src="assets/go2_hero.png" width="50%" alt="Go2 Intelligence Framework Hero Image">
  <h1>Go2 Intelligence Framework</h1>
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros&logoColor=white" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat&logo=ubuntu&logoColor=white" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/Isaac_Sim-5.1.0-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Sim 5.1.0">
  <img src="https://img.shields.io/badge/Isaac_Lab-v2.3.0-blue?style=flat&logo=nvidia&logoColor=white" alt="Isaac Lab">
  <img src="https://img.shields.io/badge/Python-3.10%20%7C%203.11-3776AB?style=flat&logo=python&logoColor=white" alt="Python Version">
  <img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=flat" alt="License">
  <p>ROS 2 & Isaac Sim based intelligence framework for Unitree Go2.</p>
</div>

---

## 🎯 Overview
The **Go2 Intelligence Framework** is an integrated autonomous system developed for the Unitree Go2 robot within the NVIDIA Isaac Sim environment. This framework bridges advanced reinforcement learning-based locomotion with robust ROS 2-based spatial intelligence (3D SLAM and Navigation). 

By providing a pre-configured pipeline of **RTAB-Map** and **Nav2**, it enables users to seamlessly transition from low-level gait control to high-level autonomous mission planning in a high-fidelity simulation, laying the groundwork for real-world Sim2Real deployment.

---

---

## 📑 Table of Contents
- [🎯 Overview](#-overview)
- [🗺️ Project Roadmap](#️-project-roadmap)
- [🛠️ Prerequisites](#️-prerequisites)
- [📦 Go2 URDF Dependency](#go2-urdf-dependency)
- [⚙️ Installation & Setup](#️-installation--setup)
- [📂 Project Structure](#-project-structure)
- [🏗️ Modules](#️-modules)
  - [0. Environment Setup](#0-environment-setup-map-customization)
  - [1. Basic Robot Simulation](#1-basic-robot-simulation-manual-control)
  - [2. 3D SLAM](#2-3d-slam-rtab-map-in-isaac-sim)
  - [3. Autonomous Navigation](#3-autonomous-navigation-nav2)
  - [4. GUI Controller](#4-gui-controller)
  - [5. Real-world Deployment](#5-real-world-deployment)
- [🤝 Acknowledgements](#-acknowledgements)
- [📄 License](#-license)

---

## 🗺️ Project Roadmap
This project aims to build a comprehensive intelligence framework for the Unitree Go2 robot through a phased development approach.

- [x] **Phase 1: 3D SLAM & Simulation (Completed)**
  - RTAB-Map integration with Isaac Sim visual/depth sensors.
  - Point cloud mapping and automated map database management.
- [x] **Phase 2: Autonomous Navigation (Completed)**
  - Integration with ROS 2 Nav2 stack for autonomous waypoint tracking.
  - Dynamic obstacle avoidance and optimized Nav2 parameter tuning.
- [x] **Phase 3: Intelligent GUI & Mission Control (Completed)**
  - Real-time telemetry dashboard & Live joint visualization.
  - Interactive mission planning with automated waypoint bridging.
- [x] **Phase 4: Real-world Hardware Deployment (Completed)**
  - **Successful Sim2Real transfer** to physical Unitree Go2 hardware.
  - Support for **STT (Speech-to-Text) & Natural Language Commands** (Predefined matching).
- [ ] **Phase 5: Advanced LLM Intelligence (Next Step)**
  - Integration of LLM-based reasoning for complex instruction tracking.
  - Advanced scene understanding for autonomous task-oriented behavior.

## 🛠️ Prerequisites
Before getting started, ensure your system meets the following requirements:

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)
- **Simulator**: [NVIDIA Isaac Sim 5.1.0](https://developer.nvidia.com/isaac-sim)
- **Framework**: [NVIDIA Isaac Lab](https://isaac-sim.github.io/IsaacLab/)
- **Python**: 3.10 or 3.11
- **Conda Environment**: Recommended (See Module Quick Start for activation)

## 📦 Go2 URDF Dependency

RViz visualization in this project requires a `go2_description` package in your ROS workspace.

Recommended references:
- https://github.com/Unitree-Go2-Robot/go2_description
- https://github.com/unitreerobotics/unitree_ros

Notes:
- `go2_description` is the most direct dependency for RViz robot visualization in this project.
- `unitree_ros` is the ROS1 repository that commonly includes description/Gazebo assets.

Install a compatible `go2_description` package and build it in your ROS workspace before launching RViz features.

---

## ⚙️ Installation & Setup
1. **Clone the repository**:
   ```bash
   git clone https://github.com/leesj24601/go2_intelligence_framework.git
   cd go2_intelligence_framework
   ```
2. **Initialize rosdep if needed**:
   ```bash
   sudo rosdep init
   rosdep update
   ```
   Skip this step if `rosdep` is already initialized on your machine.
3. **Install Python-only dependencies**:
   ```bash
   /usr/bin/python3 -m pip install --user -r requirements.txt
   ```
4. **Install ROS dependencies with rosdep**:
   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
   This reads both `go2_gui_controller` and `go2_project_dependencies` under `src/`.
5. **GUI Controller Setup (Mandatory)**:
   Before running the interactive controller for the first time, you need to set up and build the GUI package in a separate workspace:
   - **Create an external workspace and link the package**:
     ```bash
     cd ~
     mkdir -p go2_gui_controller_ws/src
     ln -s ~/go2_intelligence_framework/src/go2_gui_controller ~/go2_gui_controller_ws/src/
     ```
   - **Build the package**:
     ```bash
     cd ~/go2_gui_controller_ws
     source /opt/ros/humble/setup.bash
     colcon build --packages-select go2_gui_controller
     ```
6. **Source your ROS environments in each ROS terminal before running commands**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/go2_description_ws/install/setup.bash
   source ~/go2_gui_controller_ws/install/setup.bash
   ```

---

## 📂 Project Structure
```text
go2_intelligence_framework/
├── config/             # Configuration files for Nav2 and RViz
├── launch/             # ROS 2 launch files for SLAM and Navigation
├── maps/               # Map databases (RTAB-Map .db files)
├── policies/           # Pre-trained RL policies for Go2 locomotion
├── scripts/            # Core simulation and environment scripts
```

---

> [!IMPORTANT]
> **Execution Rule**: All commands listed below must be executed within the project's root directory (`~/go2_intelligence_framework`) unless otherwise specified.

## 🏗️ Modules

### 0. Environment Setup (Map Customization)
Both the SLAM and Navigation modules run within the same Isaac Sim environment. By default, the environment relies on a specific USD map file.

#### Creating a Custom Map (USD)
If you need to create your own simulation environment from scratch, you can build it within Isaac Sim:

<div align="center">
  <a href="https://youtu.be/74RLkOWZLKo">
    <img src="https://img.youtube.com/vi/74RLkOWZLKo/0.jpg" alt="Map Creation Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the map creation demonstration in action.</i></p>
</div>

1. Open up **NVIDIA Isaac Sim**.
2. Build your environment (add ground planes, walls, and obstacles).
3. Make sure all static colliders and ground geometry are grouped logically (for instance, under a single `/World/ground` prim).
4. Save the scene as a `.usd` file (e.g., `custom_map.usd`).
> 💡 **Automation Tip**: Alternatively, you can use **Isaac Sim MCP** with an AI assistant to automatically generate and build your 3D environment without manual placement.

#### Applying your Custom Map
To change the simulation map to your own custom USD file, you need to modify the `scripts/my_slam_env.py` file. Open the script and update the `usd_path` variable under the `MySlamEnvCfg` class:

```python
# scripts/my_slam_env.py
@configclass
class MySlamEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Change the usd_path to your custom map's location
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="usd",
            usd_path="/absolute/path/to/your/custom_map.usd", # <-- Update this line
            # ...
        )
```
> 💡 **Tip**: Make sure your custom USD file contains a proper `/World/ground` prim or adjust the `mesh_prim_paths` in the environment script. This is essential for proper physics collision and ensures the robot and sensors can interact with the environment correctly.

---

### 1. Basic Robot Simulation (Manual Control)
Before running SLAM or Navigation, you can explore your mapped environment by manually driving the Go2 robot. 

> 🧠 **Policy Info**: The locomotion of the Go2 robot is driven by a reinforcement learning policy trained via the `unitree_rl_lab` framework. You can easily swap this with your own custom-trained policy by replacing the file in the `policies/` directory, provided your policy's network structure matches.

#### 🎥 Demonstration Video
<div align="center">
  <a href="https://youtu.be/QgS4_h3jBiM">
    <img src="https://img.youtube.com/vi/QgS4_h3jBiM/0.jpg" alt="Basic Robot Simulation Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the manual control demonstration in action.</i></p>
</div>

#### 🚀 How to Run
To run the basic simulation and control the robot with your keyboard:

```bash
conda activate <isaacsim_env_name>
python scripts/go2_sim.py
```

> **Controls**: Use `W`, `A`, `S`, `D` to move and `Q`, `E` to rotate the robot.

---

### 2. 3D SLAM (RTAB-Map in Isaac Sim)
Demonstrates 3D environmental mapping using RTAB-Map with the Go2 robot within the NVIDIA Isaac Sim environment.

#### 🎥 Demonstration Video
<div align="center">
  <a href="https://youtu.be/ZbYe7EWJfB8">
    <img src="https://img.youtube.com/vi/ZbYe7EWJfB8/0.jpg" alt="RTAB-Map SLAM Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the RTAB-Map SLAM demonstration in action.</i></p>
</div>

> 💾 **Map Database Management**: 
> - **Auto-Overwrite**: In Mapping Mode, the map is saved at `maps/rtabmap.db`. This file is **overwritten** every time you restart the Mapping Mode.
> - **Saving Your Map**: Once you have created a satisfactory map, rename `maps/rtabmap.db` to **`maps/rtabmap_ground_truth.db`**.
> - **Auto-Load**: The **Localization Mode** is pre-configured to automatically load `maps/rtabmap_ground_truth.db` for stable positioning.

#### 🚀 How to Run
To run the full simulation and SLAM pipeline, please open three separate terminals.

**Terminal A**: Start the Go2 simulation environment
```bash
conda activate <isaacsim_env_name>
python scripts/go2_sim.py
```

**Terminal B**: Launch the RTAB-Map node
- **Mapping Mode** (for creating a new map):
```bash
ros2 launch launch/go2_rtabmap.launch.py
```
- **Localization Mode**:
```bash
ros2 launch launch/go2_rtabmap.launch.py localization:=true
```

**Terminal C**: Open RViz Visualization
```bash
rviz2 -d config/go2_sim.rviz
```

> 💡 **Tip**: In Localization Mode, successful localization is confirmed when the red laser scan lines perfectly align with the generated map in RViz.

---

### 3. Autonomous Navigation (Nav2)
Integration with ROS 2 Nav2 stack for autonomous waypoint navigation and obstacle avoidance within the mapped environment.

#### 🎥 Demonstration Videos

**1. Autonomous Waypoint Navigation**
<div align="center">
  <a href="https://youtu.be/J8-3K4dXg9A">
    <img src="https://img.youtube.com/vi/J8-3K4dXg9A/0.jpg" alt="Nav2 Autonomous Navigation Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the Nav2 autonomous navigation in action.</i></p>
</div>

**2. Unmapped Static Obstacle Avoidance**
<div align="center">
  <a href="https://youtu.be/W1dHQUZ_irs">
    <img src="https://img.youtube.com/vi/W1dHQUZ_irs/0.jpg" alt="Nav2 Obstacle Avoidance Demonstration" width="600">
  </a>
  <p><i>Click the image to watch the robot avoid unmapped static obstacles in real-time.</i></p>
</div>

> 🗺️ **Map Dependency**: The Nav2 module is pre-configured to automatically load the map from **`maps/rtabmap_ground_truth.db`**. Please ensure you have completed the mapping process and renamed your database file as described in the SLAM section before running navigation.

#### 🚀 How to Run
To run the Nav2 autonomous navigation, follow these steps in separate terminals.

**Terminal A**: Start the Go2 simulation environment
```bash
conda activate <isaacsim_env_name>
python scripts/go2_sim.py
```

**Terminal B**: Launch the Nav2 stack
```bash
ros2 launch launch/go2_navigation.launch.py
```

**Terminal C**: Open RViz Visualization
```bash
rviz2 -d config/go2_sim.rviz
```

> 💡 **Tip for Successful Navigation**:
> 1. **Confirm Localization First**: Successful localization is confirmed when the **red laser scan lines** perfectly align with the generated map in RViz.
> 2. **Issue Goal**: Use the `2D Goal Pose` in RViz only after localization is stable.

---

### 4. GUI Controller

#### 🎥 Demonstration Video
<div align="center">
  <a href="https://youtu.be/42z0Bue8SZ8">
    <img src="https://img.youtube.com/vi/42z0Bue8SZ8/0.jpg" alt="Complete Mission Control Dashboard & Autonomous Navigation" width="600">
  </a>
  <p><i>Click the image to watch the Complete Mission Control Dashboard & Autonomous Navigation demonstration.</i></p>
</div>

#### 🎮 Interactive Control & Monitoring Dashboard
In addition to RViz's `2D Goal Pose`, you can use the **GUI Controller** for more intuitive robot management, mission planning, and real-time monitoring.

> 🛠️ **Optimization Note**: After running the basic simulation (`python scripts/go2_sim.py`), **all other terminal-based launch commands** (SLAM, Navigation, etc.) can be fully replaced and managed through this GUI dashboard for a more streamlined experience.

*   **Intuitive Teleoperation**: Direct control via GUI buttons.
*   **Real-time Telemetry Dashboard**: Monitor the robot's state, including live charts for individual joint values directly alongside RViz.
*   **Mission Planning**: Set waypoints and monitor robot status in real-time.
*   **Natural Language Commands (Current)**: Support for predefined simple commands via text or **STT (Speech-to-Text)** voice input to automatically set corresponding waypoints via the GUI bridge.
*   **Future Update**: Full integration of LLM-based autonomous reasoning for advanced complex scene understanding and complex instruction tracking.

#### 🚀 How to Run
While `go2_sim.py` is running, open a new terminal and launch the GUI Controller:

```bash
bash scripts/run_gui_controller.sh
```
---

### 5. Real-world Deployment
This section showcases the implementation of the framework on the physical Unitree Go2 robot, demonstrating the robustness of the SLAM and navigation systems outside of the simulation.

> 💡 **Reference (LiDAR-based SLAM)**: For an alternative implementation using **4D LiDAR L1** for RTAB-Map, please refer to the following repository: [Go2_L1_Lidar_Rtabmap](https://github.com/ctrlcvlab/Go2_L1_Lidar_Rtabmap)

#### 🎥 Real-world 3D SLAM (RTAB-Map) Execution
<div align="center">
  <a href="https://youtu.be/naLCEioS-cA">
    <img src="https://img.youtube.com/vi/naLCEioS-cA/0.jpg" alt="RTAB-Map Real-world SLAM Execution" width="600">
  </a>
  <p><i>Click the image to watch the <b>3D SLAM (RTAB-Map)</b> running on the real Unitree Go2 hardware.</i></p>
</div>

#### 🚀 How to Run
To deploy on the actual Unitree Go2 robot, run the controller script to start the interface and hardware bridge:

```bash
bash scripts/run_gui_controller.sh
```

## 🤝 Acknowledgements
This project leverages several open-source libraries and frameworks:
- [Unitree Robotics](https://www.unitree.com/) for the Go2 robot and [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab).
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) for the simulation platform.
- [RTAB-Map](http://introlab.github.io/rtabmap/) for SLAM and mapping.
- [Nav2 (Navigation 2)](https://navigation.ros.org/) for autonomous navigation.
- The [ROS 2](https://www.ros.org/) community for providing the robust robotics middleware.

## 📄 License
This project is licensed under the [Apache License 2.0](LICENSE). 
You may use, modify, and distribute this software under the terms of the Apache 2.0 license. This provides additional protections for both the original authors and the users, including patent grants and mandatory attribution for modifications.
