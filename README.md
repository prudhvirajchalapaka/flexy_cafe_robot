# 🤖 Flexy Cafe Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20%7C%20Harmonic-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

An autonomous butler/cafe robot built with ROS2, featuring autonomous navigation, real-time visualization with RVIZ2, and a web-based control interface.

## 📋 Table of Contents

- [Features](#-features)
- [System Requirements](#-system-requirements)
- [Installation](#-installation)
  - [Installing ROS2 Jazzy/Harmonic](#1-install-ros2-jazzyharmonic)
  - [Installing Gazebo Harmonic](#2-install-gazebo-harmonic)
  - [Installing Dependencies](#3-install-required-packages)
  - [Building the Workspace](#4-build-the-workspace)
- [Package Structure](#-package-structure)
- [Usage](#-usage)
  - [Launching Simulation](#launching-simulation)
  - [Navigation](#navigation)
  - [Web Interface](#web-interface)
- [Configuration](#-configuration)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

## ✨ Features

- 🚀 **ROS2 Jazzy/Harmonic** support
- 🌍 **Gazebo Harmonic** simulation environment
- 🗺️ **Autonomous Navigation** using Nav2 stack
- 📊 **RVIZ2 Visualization** for real-time monitoring
- 🌐 **Web Interface** with WebSocket communication
- 🛠️ **Custom Butler Scripts** for cafe operations
- 🎯 **Waypoint Navigation** for delivery tasks
- 📡 **Real-time Telemetry** and status monitoring

## 💻 System Requirements

- **OS**: Ubuntu 22.04 (Jammy) for ROS2 Jazzy or Ubuntu 24.04 (Noble) for ROS2 Jazzy/Harmonic
- **ROS2**: Jazzy Jalisco or Harmonic Hawksbill
- **RAM**: Minimum 8GB (16GB recommended)
- **Storage**: 20GB free space
- **CPU**: Multi-core processor (4+ cores recommended)

## 🚀 Installation

### 1. Install ROS2 Jazzy/Harmonic

#### For Ubuntu 22.04 (ROS2 Jazzy):

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop-full

# Install development tools
sudo apt install ros-dev-tools
```

#### For Ubuntu 24.04 (ROS2 Harmonic):

```bash
# Follow similar steps as above, replacing 'jazzy' with 'harmonic'
sudo apt install ros-harmonic-desktop-full
```

### 2. Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic
```

### 3. Install Required Packages

```bash
# Navigation and control packages
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-gazebo-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2

# WebSocket and web interface dependencies
sudo apt install -y \
  python3-pip \
  python3-websockets \
  python3-tornado \
  python3-opencv

# Additional Python packages
pip3 install websockets aiohttp asyncio
```

### 4. Build the Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/prudhvirajchalapaka/flexy_cafe_robot.git flexy_robot

# Source ROS2
source /opt/ros/jazzy/setup.bash  # or 'harmonic' if using Harmonic

# Install dependencies using rosdep
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source ~/ros2_ws/install/setup.bash

# Add to bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 📁 Package Structure

```
flexy_robot/
├── flexy_description/       # URDF/Xacro robot description
│   ├── urdf/
│   ├── meshes/
│   └── launch/
├── flexy_gazebo/           # Gazebo simulation launch files
│   ├── launch/
│   │   ├── spawn_robot.launch.py
│   │   ├── gazebo_world.launch.py
│   │   └── robot_simulation.launch.py
│   ├── worlds/
│   └── config/
├── flexy_navigation/       # Navigation configuration
│   ├── launch/
│   │   ├── navigation.launch.py
│   │   ├── slam.launch.py
│   │   └── localization.launch.py
│   ├── config/
│   │   ├── nav2_params.yaml
│   │   └── slam_params.yaml
│   └── maps/
├── flexy_control/          # Robot control and hardware interface
│   ├── config/
│   └── launch/
├── flexy_bringup/          # Main launch files
│   ├── launch/
│   │   ├── robot.launch.py
│   │   ├── full_system.launch.py
│   │   └── rviz.launch.py
│   └── config/
├── flexy_butler/           # Butler operation scripts
│   ├── scripts/
│   │   ├── butler_manager.py
│   │   ├── delivery_planner.py
│   │   └── task_scheduler.py
│   └── launch/
└── flexy_web/              # Web interface
    ├── scripts/
    │   ├── websocket_server.py
    │   └── ros_bridge.py
    ├── web/
    │   ├── index.html
    │   ├── css/
    │   └── js/
    └── launch/
        └── web_interface.launch.py
```

## 🎮 Usage

### Launching Simulation

#### 1. Launch Gazebo World with Robot

```bash
# Terminal 1: Launch Gazebo with cafe world
ros2 launch flexy_gazebo gazebo_world.launch.py

# Terminal 2: Spawn the robot
ros2 launch flexy_gazebo spawn_robot.launch.py

# Alternative: Launch everything together
ros2 launch flexy_bringup full_system.launch.py
```

#### 2. Launch RVIZ2 Visualization

```bash
ros2 launch flexy_bringup rviz.launch.py
```

### Navigation

#### Start Navigation Stack

```bash
# Launch navigation with pre-built map
ros2 launch flexy_navigation navigation.launch.py use_sim_time:=true

# OR create a new map with SLAM
ros2 launch flexy_navigation slam.launch.py
```

#### Send Navigation Goals

**Via Command Line:**
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

**Via RVIZ2:**
1. Click "2D Goal Pose" button
2. Click and drag on the map to set goal

#### Butler Operations

```bash
# Start butler task manager
ros2 launch flexy_butler butler_manager.launch.py

# Send delivery task
ros2 run flexy_butler delivery_planner.py --table 5 --order "coffee"
```

### Web Interface

#### Start WebSocket Server

```bash
ros2 launch flexy_web web_interface.launch.py
```

#### Access Web Dashboard

Open your browser and navigate to:
```
http://localhost:8000
```

Features available in web interface:
- Live robot position tracking
- Manual robot control (WASD keys)
- Navigation goal setting
- Task queue management
- Camera feed display
- System status monitoring

## ⚙️ Configuration

### Navigation Parameters

Edit `flexy_navigation/config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_theta: 1.0
```

### Butler Task Configuration

Edit `flexy_butler/config/butler_params.yaml`:

```yaml
butler:
  max_delivery_distance: 10.0
  table_positions:
    table_1: [1.0, 2.0, 0.0]
    table_2: [3.0, 2.0, 0.0]
    # Add more tables...
```

### WebSocket Configuration

Edit `flexy_web/config/websocket_params.yaml`:

```yaml
websocket:
  host: "0.0.0.0"
  port: 8000
  update_rate: 30  # Hz
```

## 🔧 Troubleshooting

### Gazebo Won't Start

```bash
# Kill existing Gazebo processes
killall -9 gz gzclient gzserver

# Clear Gazebo cache
rm -rf ~/.gz/
```

### Navigation Not Working

```bash
# Check if all required nodes are running
ros2 node list

# Verify transforms
ros2 run tf2_tools view_frames

# Check navigation topics
ros2 topic list | grep nav
```

### Robot Not Moving

```bash
# Check velocity commands
ros2 topic echo /cmd_vel

# Verify controller status
ros2 topic echo /controller_server/status
```

### WebSocket Connection Failed

```bash
# Check if server is running
netstat -tuln | grep 8000

# Restart WebSocket server
ros2 launch flexy_web web_interface.launch.py
```

## 🐛 Common Issues

| Issue | Solution |
|-------|----------|
| `Package 'flexy_robot' not found` | Source workspace: `source ~/ros2_ws/install/setup.bash` |
| Gazebo crashes on launch | Update graphics drivers, reduce world complexity |
| Navigation unstable | Tune DWB parameters in nav2_params.yaml |
| Web interface not loading | Check firewall settings, ensure port 8000 is open |

## 📊 Performance Optimization

### For Better Simulation Performance:

```bash
# Launch with reduced visual quality
ros2 launch flexy_gazebo spawn_robot.launch.py gui:=false

# Use headless Gazebo
export LIBGL_ALWAYS_SOFTWARE=1
```

### For Real Robot Deployment:

```bash
# Launch without simulation time
ros2 launch flexy_bringup robot.launch.py use_sim_time:=false
```

## 🧪 Testing

Run unit tests:

```bash
cd ~/ros2_ws
colcon test --packages-select flexy_butler flexy_navigation
colcon test-result --verbose
```

## 📸 Screenshots & Videos

Add demo videos and screenshots to showcase your robot in action!

## 🛠️ Development

### Building Individual Packages

```bash
colcon build --packages-select flexy_gazebo
source install/setup.bash
```

### Code Style

Follow ROS2 Python style guide:
```bash
pip3 install flake8
flake8 src/flexy_robot/
```

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- **Prudhviraj Chalapaka** - [GitHub](https://github.com/prudhvirajchalapaka)

## 🙏 Acknowledgments

- ROS2 Navigation Team
- Gazebo Development Team
- Open Robotics Community

## 📧 Contact

For questions or support, please open an issue on GitHub or contact:
- Email: [prudhvirajchalapaka07@gmail.com]
- GitHub: [@prudhvirajchalapaka](https://github.com/prudhvirajchalapaka)

## 🔗 Useful Links

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [Nav2 Documentation](https://navigation.ros.org/)
- [WebSocket Protocol](https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API)

---

⭐ If you find this project helpful, please give it a star on GitHub!
