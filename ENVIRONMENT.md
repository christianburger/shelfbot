# Shelfbot — Environment Setup & Developer Guide

> **Platform:** Ubuntu 22.04 LTS (Jammy) · **ROS 2:** Humble Hawksbill · **Hardware:** 4-wheel skid-steer robot, ESP32-CAM, micro-ROS firmware

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Core Dependencies](#2-core-dependencies)
3. [ROS 2 Humble Installation](#3-ros-2-humble-installation)
4. [Workspace Setup](#4-workspace-setup)
5. [micro-ROS Setup](#5-micro-ros-setup)
6. [ORB-SLAM3 Setup (Optional)](#6-orb-slam3-setup-optional)
7. [Build the Package](#7-build-the-package)
8. [Launch the Robot](#8-launch-the-robot)
9. [RViz2 Visualisation](#9-rviz2-visualisation)
10. [Verifying Functionality with `ros2` Commands](#10-verifying-functionality-with-ros2-commands)
11. [Monitoring micro-ROS Activity](#11-monitoring-micro-ros-activity)
12. [Utility Scripts](#12-utility-scripts)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 | Ubuntu 22.04 |
| CPU | 4-core x86-64 | 8-core x86-64 |
| RAM | 8 GB | 16 GB |
| Disk | 20 GB free | 40 GB free |
| GPU | — | Any (for ORB-SLAM3 viewer) |
| Python | 3.10 | 3.10 |

**Check your Ubuntu version:**
```bash
lsb_release -a
uname -m    # must be x86_64 or aarch64
```

---

## 2. Core Dependencies

```bash
sudo apt update && sudo apt upgrade -y

# Build essentials
sudo apt install -y \
  build-essential cmake git wget curl \
  python3-pip python3-dev python3-setuptools \
  libeigen3-dev libboost-all-dev \
  libopencv-dev libopencv-contrib-dev \
  libyaml-cpp-dev libgoogle-glog-dev \
  libglew-dev libpangolin-dev \
  libapriltag-dev

# ROS 2 tool prerequisites
sudo apt install -y \
  software-properties-common \
  apt-transport-https
```

---

## 3. ROS 2 Humble Installation

```bash
# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Install desktop-full (includes RViz2, Gazebo, etc.)
sudo apt install -y ros-humble-desktop-full

# Required ROS 2 packages for shelfbot
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-hardware-interface \
  ros-humble-controller-interface \
  ros-humble-controller-manager \
  ros-humble-realtime-tools \
  ros-humble-pluginlib \
  ros-humble-nav2-bringup \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-smoother \
  ros-humble-nav2-navfn-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-costmap-2d \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-slam \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-tools \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-visualization-msgs \
  ros-humble-camera-info-manager \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-xacro \
  ros-humble-apriltag-ros \
  ros-humble-apriltag-msgs \
  ros-humble-behaviortree-cpp-v3 \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Source ROS 2 — add this to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Initialise rosdep:**
```bash
sudo rosdep init   # skip if already done
rosdep update
```

---

## 4. Workspace Setup

```bash
# Create workspace
mkdir -p ~/shelfbot_workspace/src
cd ~/shelfbot_workspace/src

# Clone the package (adjust URL/path as needed)
git clone <your-repo-url> shelfbot
# — or if working from an existing checkout —
# the package is already at ~/shelfbot_workspace/src/shelfbot

# Resolve any remaining ROS dependencies automatically
cd ~/shelfbot_workspace
rosdep install --from-paths src --ignore-src -r -y
```

---

## 5. micro-ROS Setup

The robot firmware communicates over micro-ROS topics
(`/shelfbot_firmware/motor_command`, `/shelfbot_firmware/set_speed`,
`/shelfbot_firmware/motor_positions`).  
The micro-ROS agent bridges the ESP32 serial/WiFi connection to the ROS 2 DDS layer.

### 5.1 Build the micro-ROS agent

```bash
# Create a dedicated workspace so it doesn't pollute the main build
mkdir -p ~/microros_ws/src
cd ~/microros_ws

git clone -b humble \
  https://github.com/micro-ROS/micro_ros_setup.git \
  src/micro_ros_setup

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash

# Create, build and install the micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### 5.2 Add micro-ROS to shell initialisation

```bash
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.3 Start the micro-ROS agent

The ESP32 connects via **serial USB** (most common):
```bash
# Find the correct port
ls /dev/ttyUSB* /dev/ttyACM*

# Give your user access (log out/in required once)
sudo usermod -aG dialout $USER

# Launch agent (replace /dev/ttyUSB0 and 115200 with your values)
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 \
  --baudrate 115200
```

Or over **UDP** (if the ESP32 is on WiFi):
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Keep this terminal open; the agent must be running before launching any shelfbot nodes.

---

## 6. ORB-SLAM3 Setup (Optional)

Required only when using `nav2_orb_slam3.launch.py`.  
The installation path expected by the launch file is `~/ORB_SLAM3`.

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# Install Pangolin (ORB-SLAM3 viewer dependency)
cd ~ && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd ~/ORB_SLAM3

# Patch build script for Ubuntu 22 / OpenCV 4
sed -i 's/++11/++14/g' build.sh

chmod +x build.sh
./build.sh

# Verify the shared library was produced
ls ~/ORB_SLAM3/lib/libORB_SLAM3.so
```

**Runtime environment variable** (already set in the launch file, but useful to know):
```bash
export LD_LIBRARY_PATH=~/ORB_SLAM3/lib:$LD_LIBRARY_PATH
```

---

## 7. Build the Package

### 7.1 Clean build (recommended after major changes)

```bash
cd ~/shelfbot_workspace

# Remove all previous build artefacts
rm -rf build/ install/ log/

# Source ROS 2 base (not install/setup.bash — nothing is installed yet)
source /opt/ros/humble/setup.bash

# Build everything
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source the overlay
source install/setup.bash
```

### 7.2 Incremental build (normal day-to-day)

```bash
cd ~/shelfbot_workspace
source /opt/ros/humble/setup.bash

# Build only the shelfbot package
colcon build --symlink-install --packages-select shelfbot \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
```

### 7.3 Expected build output

```
Starting >>> shelfbot_utils
Starting >>> microros_communication
Starting >>> four_wheel_drive_odometry
Starting >>> four_wheel_drive_controller
Starting >>> four_wheel_drive_hardware_interface
Starting >>> camera_publisher
Starting >>> apriltag_detector_node
Starting >>> shelfbot_slam_orb3_node   # only if ORB-SLAM3 present
Finished <<< shelfbot [XX.XXs]
Summary: X packages finished
```

No warnings about missing libraries should appear after a clean build. If ORB-SLAM3 is absent, `shelfbot_slam_orb3_node` will fail to build — this is acceptable; comment out that target in `src/nodes/CMakeLists.txt` if not needed.

---

## 8. Launch the Robot

Open **three terminals** before running any launch file.

### Terminal 1 — micro-ROS agent
```bash
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

### Terminal 2 — Robot drivers + Nav2

**Option A — Nav2 with RTAB-Map (recommended for production):**
```bash
cd ~/shelfbot_workspace
source install/setup.bash
ros2 launch shelfbot nav2_bt_real_robot.launch.py
```

**Option B — Nav2 with ORB-SLAM3:**
```bash
ros2 launch shelfbot nav2_orb_slam3.launch.py
```

**Option C — Nav2 odom-only (no SLAM, fastest for testing):**
```bash
ros2 launch shelfbot nav2_real_robot.launch.py \
  params_file:=install/shelfbot/share/shelfbot/config/nav2_odom_params.yaml
```

**Option D — AprilTag detector only:**
```bash
ros2 launch shelfbot apriltag_detector.launch.py tag_size:=0.16
```

### Terminal 3 — RViz2

```bash
source ~/shelfbot_workspace/install/setup.bash

# Full nav2 view
rviz2 -d ~/shelfbot_workspace/install/shelfbot/share/shelfbot/config/shelfbot_successful.rviz

# Odometry-only view
rviz2 -d ~/shelfbot_workspace/install/shelfbot/share/shelfbot/config/shelfbot_odometry_config.rviz

# AprilTag view
rviz2 -d ~/shelfbot_workspace/install/shelfbot/share/shelfbot/config/apriltag.rviz
```

### Sending a navigation goal from the command line

```bash
source ~/shelfbot_workspace/install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, \
    pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}}"
```

---

## 9. RViz2 Visualisation

### What each config shows

| Config file | Key displays |
|-------------|-------------|
| `shelfbot_successful.rviz` | Robot model, TF tree, global/local costmap, planner path, goal pose, camera overlay, odometry arrows |
| `shelfbot_odometry_config.rviz` | TF tree, robot model, odometry arrows only |
| `apriltag.rviz` | Raw camera feed with overlaid detections |

### Useful RViz2 panels

- **Fixed Frame** — set to `odom` for nav testing, `map` when SLAM is running
- **2D Goal Pose tool** — click on the map to send a navigation goal directly
- **Odometry display** — set *Keep* to 200 arrows to trace the driven path
- **Camera display** — toggle `Image Rendering: background and overlay` to see tags

### Adding displays manually

| Topic | Display type | Purpose |
|-------|-------------|---------|
| `/odom` | Odometry | Wheel odometry path |
| `/global_costmap/costmap` | Map | Global planner costmap |
| `/local_costmap/costmap` | Map | Local controller costmap |
| `/plan` | Path | Computed nav path |
| `/camera/image_raw` | Camera | ESP32-CAM live feed |
| `/tag_poses` | PoseArray | Detected AprilTag poses |
| `/apriltag_markers` | MarkerArray | 3D tag cubes |
| `/tf` | TF | Full transform tree |

---

## 10. Verifying Functionality with `ros2` Commands

### 10.1 Node graph

```bash
# List all running nodes
ros2 node list

# Expected nodes (with full nav2 launch):
#   /camera_info_publisher
#   /controller_manager
#   /four_wheel_drive_controller
#   /joint_state_broadcaster
#   /controller_server
#   /planner_server
#   /behavior_server
#   /bt_navigator
#   /waypoint_follower
#   /rtabmap   (or /shelfbot_slam_orb3_node)
#   /apriltag_detector
```

### 10.2 Topic inspection

```bash
# All active topics
ros2 topic list

# Verify odometry is being published (should print at ~2 Hz)
ros2 topic echo /odom --once

# Check wheel joint states
ros2 topic echo /joint_states --once

# Inspect camera feed (check timestamps are current)
ros2 topic hz /camera/image_raw

# Check camera info is publishing
ros2 topic echo /camera/camera_info --once

# Watch raw motor positions from firmware
ros2 topic echo /shelfbot_firmware/motor_positions

# Watch motor commands sent to firmware
ros2 topic echo /shelfbot_firmware/set_speed
```

### 10.3 TF tree

```bash
# Print the full transform tree to stdout
ros2 run tf2_tools view_frames

# This generates frames.pdf in the current directory showing:
# map -> odom -> base_footprint -> base_link -> [wheels, camera_link, us_*]

# Live TF lookup between two frames
ros2 run tf2_ros tf2_echo odom base_footprint

# Check if map->odom is being published (only when SLAM is running)
ros2 run tf2_ros tf2_echo map odom
```

### 10.4 Controller manager

```bash
# List all loaded controllers and their state
ros2 control list_controllers

# Expected output:
#   four_wheel_drive_controller  [active]
#   joint_state_broadcaster      [active]

# List hardware interfaces exported by the hardware plugin
ros2 control list_hardware_interfaces

# Expected: 4x velocity command, 4x position state, 4x velocity state
```

### 10.5 Hardware interface health

```bash
# Check hardware component state
ros2 control list_hardware_components

# Expected:
#   FourWheelDriveSystem [active]
```

### 10.6 Send manual velocity commands

```bash
# Forward at 0.1 m/s (CAUTION: robot will move)
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Rotate in place at 0.3 rad/s
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.3}}"

# Emergency stop
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{}"
```

### 10.7 Nav2 health check

```bash
# Check lifecycle states of Nav2 nodes
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# All should return: active

# Check action servers are ready
ros2 action list
# Expected:
#   /navigate_to_pose
#   /follow_path
#   /compute_path_to_pose
#   /spin
#   /back_up
```

### 10.8 AprilTag detection

```bash
# Verify detections are being published
ros2 topic echo /tag_poses

# Watch TF for a specific tag (e.g., tag ID 0)
ros2 run tf2_ros tf2_echo camera_link tag_0

# Count detection rate
ros2 topic hz /tag_poses
```

### 10.9 RTAB-Map / SLAM

```bash
# Check map is being built
ros2 topic echo /map --once      # occupancy grid

# Verify map->odom TF is being published (SLAM is working)
ros2 run tf2_ros tf2_echo map odom

# RTAB-Map statistics
ros2 topic echo /rtabmap/info --once
```

### 10.10 Parameter inspection

```bash
# Dump all controller parameters
ros2 param list /four_wheel_drive_controller
ros2 param get /four_wheel_drive_controller wheel_separation
ros2 param get /four_wheel_drive_controller wheel_radius

# Live-tune cmd_vel timeout (useful during debugging)
ros2 param set /four_wheel_drive_controller cmd_vel_timeout 10.0
```

---

## 11. Monitoring micro-ROS Activity

### 11.1 Confirm firmware topics appear

After starting the agent and powering the robot, these topics should appear
within ~5 seconds:

```bash
ros2 topic list | grep shelfbot_firmware
# Expected:
#   /shelfbot_firmware/motor_command
#   /shelfbot_firmware/motor_positions
#   /shelfbot_firmware/set_speed

# If topics are absent, the ESP32 has not connected to the agent.
# Check: correct port, correct baud rate, firmware flashed with micro-ROS.
```

### 11.2 Monitor connection health in real time

```bash
# Watch motor position feedback (should update at firmware control rate)
watch -n 0.5 'ros2 topic hz /shelfbot_firmware/motor_positions'

# Echo raw position values
ros2 topic echo /shelfbot_firmware/motor_positions
# data: [pos_fl, pos_fr, pos_bl, pos_br]  (float32 array, units: encoder ticks or radians)

# Echo speed commands going to firmware
ros2 topic echo /shelfbot_firmware/set_speed
# data: [vel_fl, vel_fr, vel_bl, vel_br]
```

### 11.3 Check communication health via logs

The `MicroRosCommunication` class logs a warning if no message has been
received for > 1 second.  Monitor this in the terminal running the driver launch:

```bash
# Filter relevant log lines from the launch output
ros2 launch shelfbot nav2_bt_real_robot.launch.py 2>&1 | grep -E "MicroRos|hw_positions|Communication"
```

### 11.4 Diagnose firmware silence

If `/shelfbot_firmware/motor_positions` stops publishing:

```bash
# 1. Check the agent is still running (Terminal 1)
# 2. Verify the serial device is still present
ls -la /dev/ttyUSB*

# 3. Check for permission errors
dmesg | tail -20

# 4. Restart the agent (safe to do without killing the robot)
# Ctrl-C the agent, then restart it — micro-ROS will auto-reconnect
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

---

## 12. Utility Scripts

Two helper scripts are installed to `lib/shelfbot/`:

### 12.1 `mission_starter.py` — send a sequence of waypoints

```bash
# After sourcing the workspace:
ros2 run shelfbot mission_starter.py
```

### 12.2 `topic_logger.py` — record key topics to a CSV

```bash
ros2 run shelfbot topic_logger.py
# Logs: /odom, /joint_states, /shelfbot_firmware/motor_positions
# Output: ~/shelfbot_logs/YYYY-MM-DD_HH-MM-SS.csv
```

---

## 13. Troubleshooting

### Controller not activating

```bash
ros2 control list_controllers
# If state is "inactive" or "unconfigured":

# Manually activate
ros2 control set_controller_state four_wheel_drive_controller active
ros2 control set_controller_state joint_state_broadcaster active
```

### `No motor position data received from firmware yet`

This warning means the hardware interface is alive but the firmware has not
sent a single `/shelfbot_firmware/motor_positions` message.

1. Confirm the micro-ROS agent is running and shows `[1] New Client connected`.
2. Power-cycle the ESP32/robot.
3. Verify the firmware was compiled with the correct micro-ROS topic names.

### TF tree is broken / `odom` frame missing

```bash
# Check who is broadcasting odom->base_footprint
ros2 run tf2_tools view_frames
# Open frames.pdf and look for disconnected subtrees

# If odom is missing, the odometry node in FourWheelDriveOdometry may not
# have received valid wheel positions yet.
ros2 topic echo /shelfbot_firmware/motor_positions   # must be receiving
```

### Nav2 nodes stuck in `configuring`

The lifecycle manager waits for all listed nodes to be available.
If one node crashes on startup, the entire stack stalls.

```bash
ros2 node list          # identify which node is missing
ros2 lifecycle get /planner_server   # check its state
# Review that node's stdout for error messages
```

### RTAB-Map `tf_delay` warning

If RTAB-Map logs repeated `TF Delay` warnings, the delay between odometry and
image timestamps is too large. Increase `tf_delay` in the launch file:

```python
'tf_delay': 1.2,   # seconds — raise until warnings stop
```

### Build failure: `ORB_SLAM3/lib/libORB_SLAM3.so` not found

```bash
# Either build ORB-SLAM3 (Section 6) or disable the node:
# In src/nodes/CMakeLists.txt, comment out the shelfbot_slam_orb3_node target
# and its install() entry, then rebuild.
```

### `pluginlib` cannot find controller/hardware plugin

```bash
# Verify the plugin XML files were installed
ls ~/shelfbot_workspace/install/shelfbot/share/shelfbot/*.xml

# Re-index the plugin cache
ros2 pkg list | grep shelfbot    # package must appear
```

---

## Quick-Reference Cheat Sheet

```bash
# ── Build ────────────────────────────────────────────────────────────────────
cd ~/shelfbot_workspace && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select shelfbot && source install/setup.bash

# ── Run (3 terminals) ────────────────────────────────────────────────────────
# T1: micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200

# T2: robot stack
ros2 launch shelfbot nav2_bt_real_robot.launch.py

# T3: RViz2
rviz2 -d ~/shelfbot_workspace/install/shelfbot/share/shelfbot/config/shelfbot_successful.rviz

# ── Health checks ────────────────────────────────────────────────────────────
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic hz /shelfbot_firmware/motor_positions
ros2 run tf2_tools view_frames && evince frames.pdf

# ── Drive test ───────────────────────────────────────────────────────────────
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
