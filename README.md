🤖 Autonomous Indoor Navigation Robot

Full-stack autonomous indoor navigation system with 3D mapping, A* planning, multi-sensor fusion, and real-time web monitoring.

🎯 Overview

This project implements a complete ROS2 Humble autonomous navigation stack featuring:

3D LiDAR mapping with voxel filtering

2D costmap generation from 3D point cloud

Custom A* global planner

Potential Field local controller

Multi-layer safety system (LiDAR + depth camera)

Smooth acceleration-limited motion

Interactive web UI dashboard

Built for research, simulation, and real-world AMR deployment.

🖥️ System Demonstration
<img width="1366" height="768" alt="Screenshot from 2026-02-11 22-59-43" src="https://github.com/user-attachments/assets/673c1ae3-fc34-4d8f-97a1-40bd86186cc6" />
🌐 Web Dashboard

Real-time monitoring, goal setting, safety status, and teleoperation.

<img width="1366" height="768" alt="Screenshot from 2026-02-11 23-00-32" src="https://github.com/user-attachments/assets/736553c1-21ef-49c8-84ff-ec3de81d1ee4" />
🗺️ Gazebo Simulation

LiDAR scanning and real-time perception in simulation.

<img width="1366" height="768" alt="Screenshot from 2026-02-11 22-59-59" src="https://github.com/user-attachments/assets/ad7083c3-f994-4cce-b56e-49be4ced7b4c" />
📡 System Logs (Planning + Mapping)

A* planner receiving costmap and publishing path.
<img width="1366" height="768" alt="Screenshot from 2026-02-11 22-58-43" src="https://github.com/user-attachments/assets/e6576566-5557-446e-82e9-cbbada57047f" />
🧭 RViz Visualization

3D environment, planned path, costmap, and robot pose visualization.

✨ Key Features
🗺 Mapping & Perception

3D point cloud accumulation

Self-filtering (robot body removal)

Ground removal

Map save/load (.pcd)

3D → 2D costmap conversion
<img width="1366" height="768" alt="Screenshot from 2026-02-11 23-10-48" src="https://github.com/user-attachments/assets/d9c3c579-3dce-4447-8ccb-a8d082dfcca9" />

🧭 Navigation

Custom A* global planner

Potential Field local planner

Adaptive lookahead (0.5m – 1.5m)

Smooth velocity profiling

Acceleration limiting

🛡 Safety System

3-tier safety zones:

🔴 0–0.4m → Emergency Stop

🟡 0.4–0.8m → 30% speed

🟢 0.8–1.2m → 60% speed

Additional features:

50Hz safety monitoring

Gradual braking

Multi-sensor fusion

🏗 System Architecture
Web UI / RViz
       ↓
A* Global Planner
       ↓
Local Planner (Potential Field)
       ↓
Safety Monitor
       ↓
Robot (Gazebo / Real Hardware)

Perception:
LiDAR → 3D Map → 2D Costmap
Depth Camera → Safety Layer
📦 Requirements

Ubuntu 22.04

ROS2 Humble

Gazebo Classic

GCC 11+ (C++17)

Install dependencies:

sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-pcl-ros \
  ros-humble-rosbridge-server \
  python3-colcon-common-extensions
🚀 Installation
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/karadasrikarreddy/Web-UI-Controlled-Robot

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
🎮 Usage

You can operate the system either via terminal or entirely from the Web UI.

🗺 First-Time Mapping (Terminal)
ros2 launch my_robot_controller mapping_3d.launch.py
ros2 run my_robot_teleop teleop_node
ros2 service call /save_map_3d std_srvs/srv/Trigger
🌐 Mapping via Web Dashboard

From the Web UI you can:

🗺 Create a New Map

Click "Create New Map"

System switches to mapping mode

Drive robot using teleop controls in UI

💾 Save Map

Enter map name

Click "Save Map"

Map saved as .pcd

🤖 Autonomous Navigation
ros2 launch my_robot_controller indoor.launch.py

Then open the Web UI:

cd ~/ros2_ws/web_ui
python3 -m http.server 8000

Open in browser:

http://localhost:8000

Click anywhere on the map to send navigation goals.

⚙ Configuration

Main tuning file:

config/safety_params.yaml

Key adjustable parameters:

max_speed

lookahead_dist

obstacle_weight

critical_distance

📁 Project Structure
src/
 
 ├── my_robot_controller/
 
 │   ├── a_star_planner.cpp
 
 │   ├── local_planner.cpp
 
 │   ├── safety_monitor.cpp
 
 │   └── mapping nodes
 
 ├── my_robot_description/
 
 ├── my_robot_vision/ 
 
 └── my_robot_teleop/

web_ui/
docs/
🔬 Technical Highlights

Control Loop: 20 Hz

Safety Monitoring: 50 Hz

Voxel Resolution: 0.05m

Adaptive Lookahead

Costmap Inflation

Diagonal A* Expansion

🗺 Roadmap
v1.0

3D Mapping

A* Planner

Local Controller

Safety System

Web UI

Next

Dynamic obstacle tracking

Re-planning

Multi-goal navigation

👤 Author

Srikar Reddy
Robotics & Autonomous Systems Engineer

📜 License

Apache License 2.0
