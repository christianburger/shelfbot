# Shelfbot — Environment Setup & Developer Guide (Native / Bare-Metal)

> **Platform:** Ubuntu 22.04 LTS (Jammy) · **ROS 2:** Humble Hawksbill
> **Hardware:** 4-wheel skid-steer robot · ESP32-CAM · micro-ROS firmware

> ⚠️ **Docker is the recommended environment.**
> See [`INSTRUCTIONS.md`](INSTRUCTIONS.md) for the Docker-based workflow, which
> gives a fully reproducible build (including ORB-SLAM3 and Pangolin) without
> modifying host packages.
> This document covers native installation on a bare-metal Ubuntu 22.04 machine.

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
10. [Verifying Functionality](#10-verifying-functionality)
11. [Monitoring micro-ROS Activity](#11-monitoring-micro-ros-activity)
12. [Utility Scripts](#12-utility-scripts)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |
| CPU | 4-core x86-64 | 8-core x86-64 |
| RAM | 8 GB | 16 GB |
| Disk | 20 GB free | 40 GB free |
| GPU | — | Any (for ORB-SLAM3 Pangolin viewer) |
| Python | 3.10 | 3.10 |

```bash
lsb_release -a
uname -m    # must be x86_64 or aarch64
```

---

## 2. Core Dependencies

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  build-essential cmake git wget curl unzip ninja-build \
  python3-pip python3-dev python3-setuptools \
  libeigen3-dev libboost-all-dev \
  libopencv-dev libopencv-contrib-dev \
  libyaml-cpp-dev libgoogle-glog-dev \
  libglew-dev libgl1-mesa-dev \
  libapriltag-dev \
  software-properties-common apt-transport-https
```

---

## 3. ROS 2 Humble Installation

```bash
# Add ROS 2 APT key and repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Desktop-full (includes RViz2, rqt)
sudo apt install -y ros-humble-desktop-full

# Shelfbot-specific ROS 2 packages
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
  ros-humble-image-transport-plugins \
  ros-humble-cv-bridge \
  ros-humble-xacro \
  ros-humble-apriltag-ros \
  ros-humble-apriltag-msgs \
  ros-humble-rosidl-typesupport-c \
  ros-humble-rosidl-typesupport-cpp \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Source ROS 2 in every new shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Initialise rosdep (skip if already done):

```bash
sudo rosdep init
rosdep update
```

---

## 4. Workspace Setup

The repo root acts as the ROS 2 package source. The colcon workspace lives
inside the repo at `shelfbot_ws/`.

```bash
cd /home/chris/shelfbot/shelfbot

# The colcon workspace sits inside the repo
mkdir -p shelfbot_ws/src

# Symlink the package source into the workspace
# Use an absolute path so the symlink works from both native and Docker contexts
ln -sf "$(pwd)" shelfbot_ws/src/shelfbot

# Resolve remaining ROS dependencies
cd shelfbot_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **Docker note:** The Docker setup uses a second bind-mount instead of a
> symlink (absolute symlinks break inside containers). If you switch between
> native and Docker workflows, the bind-mount shadows any symlink, so both
> can coexist.

---

## 5. micro-ROS Setup

The robot firmware communicates over micro-ROS topics:
- `/shelfbot_firmware/motor_command`
- `/shelfbot_firmware/set_speed`
- `/shelfbot_firmware/motor_positions`

The micro-ROS agent bridges the ESP32 serial/Wi-Fi connection to ROS 2 DDS.

### 5.1 Build the micro-ROS agent

```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws

git clone -b humble \
  https://github.com/micro-ROS/micro_ros_setup.git \
  src/micro_ros_setup

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### 5.2 Add to shell initialisation

```bash
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.3 Start the agent

Serial (most common):

```bash
ls /dev/ttyUSB* /dev/ttyACM*          # find the device
sudo usermod -aG dialout $USER         # one-time; requires logout/login

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

UDP (ESP32 on Wi-Fi):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Keep this terminal open. The agent must be running before any shelfbot node
that uses the hardware interface.

---

## 6. ORB-SLAM3 Setup (Optional)

Required only for `nav2_orb_slam3.launch.py`.
In the Docker image, ORB-SLAM3 is at `/opt/ORB_SLAM3`.
For a native build, install it to a known path and export the variable.

### 6.1 Build Pangolin

```bash
cd ~
git clone --depth 1 --branch v0.6 \
  https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYPANGOLIN=OFF
make -j$(nproc)
sudo make install        # installs to /usr/local
```

### 6.2 Build ORB-SLAM3

```bash
cd ~
git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# Verify
ls ~/ORB_SLAM3/lib/libORB_SLAM3.so
```

### 6.3 Export environment variable

```bash
echo "export ORB_SLAM3_ROOT=~/ORB_SLAM3" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$ORB_SLAM3_ROOT/lib:\$ORB_SLAM3_ROOT/Thirdparty/DBoW2/lib:\$ORB_SLAM3_ROOT/Thirdparty/g2o/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

> `src/nodes/CMakeLists.txt` reads `$ENV{ORB_SLAM3_ROOT}` and falls back to
> `/opt/ORB_SLAM3` if the variable is not set.
> Pangolin is searched at `/opt/pangolin` first, then the CMake default paths.
> For a native build where Pangolin is in `/usr/local`, you may need to set
> `CMAKE_PREFIX_PATH=/usr/local` or adjust the `find_library` paths.

---

## 7. Build the Package

### 7.1 Clean build (recommended after major changes)

```bash
cd /home/chris/shelfbot/shelfbot/shelfbot_ws

rm -rf build/ install/ log/

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
```

### 7.2 Incremental build (day-to-day)

```bash
cd /home/chris/shelfbot/shelfbot/shelfbot_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select shelfbot \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
```

### 7.3 Expected build output

```
Starting >>> shelfbot
--- stderr: shelfbot
[Eigen::AlignedBit deprecation warnings — see §13 → Known warnings]
---
Finished <<< shelfbot [~50s]

Summary: 1 package finished [~50s]
  1 package had stderr output: shelfbot
```

`Summary: 1 package finished` with no `Failed` line is a **successful build**.
The stderr content is entirely harmless Eigen deprecation warnings produced
by ORB-SLAM3's bundled g2o library — see [§13](#13-troubleshooting).

If ORB-SLAM3 is not installed and `$ORB_SLAM3_ROOT` is not set, the
`shelfbot_slam_orb3_node` target will fail. Disable it by commenting out its
`add_executable` block and `install()` entry in `src/nodes/CMakeLists.txt`.

---

## 8. Launch the Robot

Open three terminals. Source the workspace in each:

```bash
source /home/chris/shelfbot/shelfbot/shelfbot_ws/install/setup.bash
```

### Terminal 1 — micro-ROS agent

```bash
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

### Terminal 2 — Robot stack

```bash
cd /home/chris/shelfbot/shelfbot/shelfbot_ws
source install/setup.bash

# Option A — Nav2 + RTAB-Map SLAM (recommended)
ros2 launch shelfbot nav2_bt_real_robot.launch.py

# Option B — Nav2 + ORB-SLAM3
ros2 launch shelfbot nav2_orb_slam3.launch.py

# Option C — Nav2, odometry only
ros2 launch shelfbot nav2_real_robot.launch.py

# Option D — AprilTag detector only
ros2 launch shelfbot apriltag_detector.launch.py tag_size:=0.16
```

### Terminal 3 — RViz2

```bash
source /home/chris/shelfbot/shelfbot/shelfbot_ws/install/setup.bash

rviz2 -d /home/chris/shelfbot/shelfbot/shelfbot_ws/install/shelfbot/share/shelfbot/config/shelfbot_successful.rviz
```

### Send a navigation goal

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, \
    pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}}"
```

---

## 9. RViz2 Visualisation

| Config file | Key displays |
|-------------|-------------|
| `shelfbot_successful.rviz` | Robot model, TF tree, global/local costmap, planner path, goal pose, camera overlay, odometry arrows |
| `shelfbot_odometry_config.rviz` | TF tree, robot model, odometry arrows only |
| `apriltag.rviz` | Raw camera feed with overlaid detections |

**Tips:**
- Set Fixed Frame to `odom` for nav testing, `map` when SLAM is running
- Use the **2D Goal Pose** tool to send nav goals by clicking the map
- Set Odometry *Keep* to 200 to trace the driven path

| Topic | Display type | Purpose |
|-------|-------------|---------|
| `/odom` | Odometry | Wheel odometry path |
| `/global_costmap/costmap` | Map | Global planner costmap |
| `/local_costmap/costmap` | Map | Local controller costmap |
| `/plan` | Path | Computed nav path |
| `/camera/image_raw` | Camera | ESP32-CAM live feed |
| `/tag_poses` | PoseArray | Detected AprilTag poses |
| `/apriltag_markers` | MarkerArray | 3D tag cubes in RViz |
| `/tf` | TF | Full transform tree |

---

## 10. Verifying Functionality

### Node graph

```bash
ros2 node list
# With full nav2_bt_real_robot.launch.py:
#   /four_wheel_drive_controller
#   /joint_state_broadcaster
#   /controller_manager
#   /controller_server
#   /planner_server
#   /behavior_server
#   /bt_navigator
#   /waypoint_follower
#   /rtabmap   (or /shelfbot_slam_orb3_node)
#   /apriltag_detector_node
```

### Topics

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 topic hz /camera/image_raw
ros2 topic echo /shelfbot_firmware/motor_positions
ros2 topic echo /shelfbot_firmware/set_speed
```

### TF tree

```bash
ros2 run tf2_tools view_frames    # generates frames.pdf
# Chain: map → odom → base_footprint → base_link → wheels, camera_link

ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom   # only when SLAM is active
```

### Controller manager

```bash
ros2 control list_controllers
# four_wheel_drive_controller  [active]
# joint_state_broadcaster      [active]

ros2 control list_hardware_interfaces
# 4× velocity command, 4× position state, 4× velocity state

ros2 control list_hardware_components
# FourWheelDriveSystem [active]
```

### Manual velocity (robot will move — caution)

```bash
# Forward 0.1 m/s
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Rotate 0.3 rad/s
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}"

# Stop
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{}"
```

### Nav2 health

```bash
ros2 lifecycle get /controller_server   # → active
ros2 lifecycle get /planner_server      # → active
ros2 lifecycle get /bt_navigator        # → active

ros2 action list
# /navigate_to_pose  /follow_path  /compute_path_to_pose  /spin  /back_up
```

### AprilTag detection

```bash
ros2 topic echo /tag_poses
ros2 topic hz /tag_poses
ros2 run tf2_ros tf2_echo camera_link tag_0
```

### RTAB-Map / SLAM

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /rtabmap/info --once
```

### Parameter inspection

```bash
ros2 param list /four_wheel_drive_controller
ros2 param get /four_wheel_drive_controller wheel_separation
ros2 param get /four_wheel_drive_controller wheel_radius
ros2 param set /four_wheel_drive_controller cmd_vel_timeout 10.0
```

---

## 11. Monitoring micro-ROS Activity

### Confirm firmware topics appear

Within ~5 seconds of powering the robot and starting the agent:

```bash
ros2 topic list | grep shelfbot_firmware
# /shelfbot_firmware/motor_command
# /shelfbot_firmware/motor_positions
# /shelfbot_firmware/set_speed
```

If topics are absent: check the port, baud rate, and that the firmware is
flashed with micro-ROS.

### Real-time monitoring

```bash
watch -n 0.5 'ros2 topic hz /shelfbot_firmware/motor_positions'

ros2 topic echo /shelfbot_firmware/motor_positions
# data: [pos_fl, pos_fr, pos_bl, pos_br]

ros2 topic echo /shelfbot_firmware/set_speed
# data: [vel_fl, vel_fr, vel_bl, vel_br]
```

### Diagnose firmware silence

```bash
ls -la /dev/ttyUSB*
dmesg | tail -20

# Restart agent (micro-ROS auto-reconnects)
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

---

## 12. Utility Scripts

### `mission_starter.py` — send a sequence of waypoints

```bash
ros2 run shelfbot mission_starter.py
```

### `topic_logger.py` — record key topics to CSV

```bash
ros2 run shelfbot topic_logger.py
# Logs /odom, /joint_states, /shelfbot_firmware/motor_positions
# Output: ~/shelfbot_logs/YYYY-MM-DD_HH-MM-SS.csv
```

---

## 13. Troubleshooting

### Known build warnings: `Eigen::AlignedBit` is deprecated

```
warning: 'Eigen::AlignedBit' is deprecated [-Wdeprecated-declarations]
  typedef Eigen::Map<..., AlignedBit ? Aligned : Unaligned> HessianBlockType;
```

**These are harmless.** Source: ORB-SLAM3's bundled g2o library, written
against Eigen 3.3. Ubuntu 22.04 ships Eigen 3.4 which deprecated `AlignedBit`.
The compiled binary is correct. They are suppressed by adding to
`src/nodes/CMakeLists.txt`:

```cmake
target_compile_options(shelfbot_slam_orb3_node PRIVATE
  -Wno-deprecated-declarations
)
```

Cannot be fixed without patching upstream g2o.

### Controller not activating

```bash
ros2 control list_controllers   # check state
ros2 control set_controller_state four_wheel_drive_controller active
ros2 control set_controller_state joint_state_broadcaster active
```

### `No motor position data received from firmware yet`

1. Confirm micro-ROS agent shows `[1] New Client connected`
2. Power-cycle the ESP32
3. Verify firmware is compiled with the correct micro-ROS topic names

### TF tree broken / `odom` frame missing

```bash
ros2 run tf2_tools view_frames   # open frames.pdf for disconnected subtrees
ros2 topic echo /shelfbot_firmware/motor_positions   # must be receiving
```

### Nav2 nodes stuck in `configuring`

One node crashed on startup — the lifecycle manager waits for it.

```bash
ros2 node list                      # identify the missing node
ros2 lifecycle get /planner_server  # check its state
```

### RTAB-Map `TF Delay` warning

```python
# In the launch file, increase:
'tf_delay': 1.2,   # seconds
```

### Build failure: `libORB_SLAM3.so` not found

Either build ORB-SLAM3 (§6), set `ORB_SLAM3_ROOT`, or disable the node:

```cmake
# In src/nodes/CMakeLists.txt — comment out:
# add_executable(shelfbot_slam_orb3_node ...)
# target_include_directories(shelfbot_slam_orb3_node ...)
# target_link_libraries(shelfbot_slam_orb3_node ...)
# ament_target_dependencies(shelfbot_slam_orb3_node ...)
# install(TARGETS shelfbot_slam_orb3_node ...)
```

### `pluginlib` cannot find controller/hardware plugin

```bash
ls /home/chris/shelfbot/shelfbot/shelfbot_ws/install/shelfbot/share/shelfbot/*.xml
ros2 pkg list | grep shelfbot   # package must appear
```

---

## Quick-Reference Cheat Sheet

```bash
# ── Docker workflow (recommended) ─────────────────────────────────────────────
cd /home/chris/shelfbot/shelfbot
docker compose up -d
docker compose exec shelfbot bash
# Inside container:
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash

# ── Native workflow ───────────────────────────────────────────────────────────
cd /home/chris/shelfbot/shelfbot/shelfbot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select shelfbot \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# ── Launch (3 terminals) ──────────────────────────────────────────────────────
# T1: micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200

# T2: robot stack
ros2 launch shelfbot nav2_bt_real_robot.launch.py

# T3: RViz2
rviz2 -d $(ros2 pkg prefix shelfbot)/share/shelfbot/config/shelfbot_successful.rviz

# ── Health checks ─────────────────────────────────────────────────────────────
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic hz /shelfbot_firmware/motor_positions
ros2 run tf2_tools view_frames

# ── Drive test ────────────────────────────────────────────────────────────────
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
