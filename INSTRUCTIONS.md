# Shelfbot — Docker Setup & Operations Guide

Docker is the **recommended** development and deployment environment.
It provides a fully reproducible ROS 2 Humble system — including Pangolin and
ORB-SLAM3 compiled from source — without touching host packages.

For native (bare-metal) installation see [`ENVIRONMENT.md`](ENVIRONMENT.md).

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Repository layout and bind-mount strategy](#2-repository-layout-and-bind-mount-strategy)
3. [One-time host setup](#3-one-time-host-setup)
4. [Building the Docker image](#4-building-the-docker-image)
5. [Starting the container](#5-starting-the-container)
6. [Opening shells](#6-opening-shells)
7. [Building the ROS 2 package](#7-building-the-ros-2-package)
8. [Launching the robot](#8-launching-the-robot)
9. [Stopping and re-entering](#9-stopping-and-re-entering)
10. [Rebuilding the image from scratch](#10-rebuilding-the-image-from-scratch)
11. [CLion IDE integration](#11-clion-ide-integration)
12. [Known build warnings](#12-known-build-warnings)
13. [Verified dependency list](#13-verified-dependency-list)
14. [Manual verification checklist](#14-manual-verification-checklist)

---

## 1. Prerequisites

| Requirement | Minimum version | Notes |
|---|---|---|
| Docker Engine | 24.x | `curl -fsSL https://get.docker.com \| sh` |
| Docker Compose plugin | 2.x | `docker compose` (no hyphen) |
| Host user UID/GID | 1000 / 1000 | The image creates user `chris` with these IDs; files stay writable on both sides of the bind-mount |
| NVIDIA driver | 525+ | Optional — only needed for GPU-accelerated ORB-SLAM3 viewer |

```bash
# Install Docker and add your user to the docker group
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # log out and back in after this
```

---

## 2. Repository layout and bind-mount strategy

```
/home/chris/shelfbot/shelfbot/        ← repo root  (where docker compose is run)
├── Dockerfile
├── docker-compose.yml
├── docker-entrypoint.sh              ← auto-repairs /workspace ownership on every start
├── .env                              ← DOCKER_UID=1000 / DOCKER_GID=1000
├── .devcontainer/devcontainer.json
├── CMakePresets.json
├── ... (all package source files)
└── shelfbot_ws/                      ← colcon workspace; must exist before docker compose up
    ├── build/                        ← created by colcon
    ├── install/                      ← created by colcon
    └── log/                          ← created by colcon
```

Two bind-mounts are active at runtime:

| Host path | Container path | Purpose |
|---|---|---|
| `./shelfbot_ws` | `/workspace` | Colcon workspace — `build/`, `install/`, `log/` live here and persist on the host |
| `.` (repo root) | `/workspace/src/shelfbot` | Package source — edits in the repo are immediately visible inside the container |

> **No symlink is needed.** The second mount provides `src/shelfbot` directly.
> The old `shelfbot_ws/src/shelfbot` symlink approach broke inside the container
> because symlinks with absolute host paths resolve incorrectly across the mount boundary.

### Why `shelfbot_ws` must exist before `docker compose up`

If `./shelfbot_ws` is absent when `docker compose up` runs, Docker creates it
automatically — but as **root**. The container's `chris` user (UID 1000) then
cannot write to it. The `docker-entrypoint.sh` script detects and repairs this
automatically using `sudo` (chris has `NOPASSWD` sudo inside the container),
but it is cleaner and faster to pre-create the directory:

```bash
mkdir -p shelfbot_ws   # run once from the repo root on the host
```

---

## 3. One-time host setup

Run these commands **once** from the repo root on the host machine:

```bash
cd /home/chris/shelfbot/shelfbot

# Create the colcon workspace directory as chris (prevents Docker creating it as root)
mkdir -p shelfbot_ws

# Allow container GUI apps (RViz2, rqt) to use the host X11 display
xhost +local:docker
```

The `.env` file already exists in the repo and contains:

```dotenv
DOCKER_UID=1000
DOCKER_GID=1000
```

> **Note:** `UID` is a readonly variable in bash and cannot be assigned on the
> command line (`UID=$(id -u)` fails). Docker Compose reads `DOCKER_UID` and
> `DOCKER_GID` from `.env` automatically — no prefix is needed on any command.

---

## 4. Building the Docker image

### 4a. Full build (includes ORB-SLAM3) — recommended

> ⚠️ Takes **10–20 minutes** the first time. Pangolin v0.6 and ORB-SLAM3 are
> compiled from source inside the image and installed to `/opt/pangolin` and
> `/opt/ORB_SLAM3` respectively.

```bash
docker compose build
```

### 4b. Development build (skip ORB-SLAM3, much faster)

Edit `docker-compose.yml` and uncomment the `target: base` line:

```yaml
build:
  context: .
  dockerfile: Dockerfile
  target: base    # ← uncomment this line
```

Then:

```bash
docker compose build
```

With `target: base`, the `shelfbot_slam_orb3_node` target will fail to compile.
Disable it by commenting out the `shelfbot_slam_orb3_node` block in
`src/nodes/CMakeLists.txt` before building the package.

---

## 5. Starting the container

```bash
docker compose up -d
```

The container starts as user `chris` (UID 1000). On first start the entrypoint
script checks `/workspace` ownership and repairs it if Docker created the
directory as root.

To verify the container is running:

```bash
docker compose ps
# NAME        IMAGE             STATUS
# shelfbot    shelfbot:humble   Up N seconds
```

---

## 6. Opening shells

```bash
# Primary shell
docker compose exec shelfbot bash

# Additional shells (open in new terminals as needed)
docker compose exec shelfbot bash
```

The shell opens as `chris` in `/workspace`. The ROS 2 environment
(`/opt/ros/humble/setup.bash`) and the workspace overlay
(`/workspace/install/setup.bash`) are sourced automatically from
`/home/chris/.bashrc`.

---

## 7. Building the ROS 2 package

Run inside the container:

```bash
cd /workspace

# First-time only: install any rosdep dependencies not in the image
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source the overlay (also done automatically in new shells via .bashrc)
source install/setup.bash
```

### Expected output

```
Starting >>> shelfbot
--- stderr: shelfbot
[Eigen::AlignedBit deprecation warnings — see §12]
---
Finished <<< shelfbot [~50s]

Summary: 1 package finished [~50s]
  1 package had stderr output: shelfbot
```

`Summary: 1 package finished` with no `Failed` line is a **successful build**.
The stderr output is entirely deprecation warnings from ORB-SLAM3's bundled
g2o library — see [§12](#12-known-build-warnings).

### Incremental builds (day-to-day)

```bash
colcon build --symlink-install --packages-select shelfbot \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Clean build (after major structural changes)

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

---

## 8. Launching the robot

Open **three container shells** (`docker compose exec shelfbot bash` in separate
terminals).

### Shell 1 — micro-ROS agent

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyUSB0 --baudrate 115200
```

Or over UDP (if the ESP32 is on Wi-Fi):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Keep this shell open. The agent must be running before the robot stack is launched.

### Shell 2 — Robot stack

```bash
source /workspace/install/setup.bash

# Option A — Nav2 + RTAB-Map SLAM (recommended for production)
ros2 launch shelfbot nav2_bt_real_robot.launch.py

# Option B — Nav2 + ORB-SLAM3 (requires full image build)
ros2 launch shelfbot nav2_orb_slam3.launch.py

# Option C — Nav2, wheel odometry only (fastest for testing)
ros2 launch shelfbot nav2_real_robot.launch.py

# Option D — AprilTag detector only
ros2 launch shelfbot apriltag_detector.launch.py tag_size:=0.16
```

### Shell 3 — RViz2

```bash
source /workspace/install/setup.bash
rviz2 -d /workspace/install/shelfbot/share/shelfbot/config/shelfbot_successful.rviz
```

Other configs:

```bash
# Odometry only
rviz2 -d /workspace/install/shelfbot/share/shelfbot/config/shelfbot_odometry_config.rviz

# AprilTag view
rviz2 -d /workspace/install/shelfbot/share/shelfbot/config/apriltag.rviz
```

### Sending a navigation goal from the command line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, \
    pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
           orientation: {w: 1.0}}}}"
```

### Manual velocity commands (robot will move — caution)

```bash
# Forward at 0.1 m/s
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Rotate in place at 0.3 rad/s
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}"

# Emergency stop
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{}"
```

---

## 9. Stopping and re-entering

```bash
# Stop (workspace data is preserved on the host in shelfbot_ws/)
docker compose down

# Restart
docker compose up -d
docker compose exec shelfbot bash
```

The `install/setup.bash` source in `.bashrc` is a no-op if the file does not
exist yet (redirected to `/dev/null`), so new shells are safe before the first
build.

---

## 10. Rebuilding the image from scratch

```bash
docker compose down
docker compose build --no-cache
docker compose up -d
```

To also wipe the colcon workspace artefacts on the host:

```bash
docker compose down
sudo rm -rf shelfbot_ws/build shelfbot_ws/install shelfbot_ws/log
# Do NOT rm shelfbot_ws itself — it would be recreated as root by Docker
docker compose up -d
```

To remove the image entirely and start over:

```bash
docker compose down --rmi all
mkdir -p shelfbot_ws   # recreate before building
docker compose build
docker compose up -d
```

---

## 11. CLion IDE integration

CLion Gateway (RemoteDev) connects to the running container over SSH.

### devcontainer.json (`/.devcontainer/devcontainer.json`)

`workspaceFolder` is set to `/workspace/src/shelfbot` (the package source),
not `/workspace` (the colcon workspace root), so CLion opens the package's
`CMakeLists.txt` directly. Both `remoteUser` and `containerUser` are `chris`.

### CMakePresets.json (repo root)

Three presets are provided:

| Preset name | Use when |
|---|---|
| `ros2-humble-debug` | Normal development after first `colcon build` |
| `ros2-humble-release` | Release builds |
| `ros2-humble-debug-no-install` | First time, before `colcon build` has run |

Select in CLion: **Settings → Build, Execution, Deployment → CMake → CMake preset**.

The presets set `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `LD_LIBRARY_PATH`,
and `ROS_DISTRO` in the CMake process environment so that `find_package` and
ament index lookups work without sourcing a shell script.

### Workflow

1. `docker compose up -d` (container must be running)
2. In CLion Gateway, connect to the running container and open
   `/workspace/src/shelfbot` as the project root
3. Select the `ros2-humble-debug-no-install` preset for the first CMake load
4. Run `colcon build` in an exec shell (see §7)
5. Switch to the `ros2-humble-debug` preset and reload CMake

---

## 12. Known build warnings

Building `shelfbot_slam_orb3_node` produces many lines like:

```
warning: 'Eigen::AlignedBit' is deprecated [-Wdeprecated-declarations]
  typedef Eigen::Map<..., ..., AlignedBit ? Aligned : Unaligned> HessianBlockType;
```

**These are harmless.** They originate entirely inside ORB-SLAM3's bundled g2o
third-party library (`/opt/ORB_SLAM3/Thirdparty/g2o/`) which was written
against Eigen 3.3. Ubuntu 22.04 ships Eigen 3.4, which deprecated `AlignedBit`.
The compiled binary is correct and fully functional.

The warnings are suppressed in `src/nodes/CMakeLists.txt`:

```cmake
target_compile_options(shelfbot_slam_orb3_node PRIVATE
  -Wno-deprecated-declarations   # Eigen::AlignedBit from ORB-SLAM3/g2o third-party
)
```

They cannot be fixed without patching the upstream g2o source.

---

## 13. Verified dependency list

All packages below are installed in the Docker image and verified against the
Ubuntu 22.04 (jammy) ROS 2 Humble APT repository.

### Core rclcpp
- `ros-humble-rclcpp`, `ros-humble-rclcpp-lifecycle`

### Messages
- `ros-humble-std-msgs`, `ros-humble-sensor-msgs`, `ros-humble-geometry-msgs`
- `ros-humble-nav-msgs`, `ros-humble-visualization-msgs`
- `ros-humble-builtin-interfaces`

### TF2
- `ros-humble-tf2`, `ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs`

### ros2\_control
- `ros-humble-ros2-control`, `ros-humble-ros2-controllers`
- `ros-humble-controller-interface`, `ros-humble-hardware-interface`
- `ros-humble-realtime-tools`, `ros-humble-pluginlib`

### Robot description
- `ros-humble-xacro`, `ros-humble-robot-state-publisher`
- `ros-humble-joint-state-publisher`, `ros-humble-joint-state-publisher-gui`

### Visualisation
- `ros-humble-rviz2`

### Vision / Perception
- `ros-humble-image-transport`, `ros-humble-image-transport-plugins`
- `ros-humble-cv-bridge`, `ros-humble-camera-info-manager`
- `ros-humble-message-filters`
- `ros-humble-apriltag-ros`, `ros-humble-apriltag-msgs`

### Navigation 2
- `ros-humble-nav2-bringup`, `ros-humble-nav2-controller`
- `ros-humble-nav2-planner`, `ros-humble-nav2-map-server`
- `ros-humble-nav2-lifecycle-manager`, `ros-humble-nav2-common`
- `ros-humble-nav2-bt-navigator`, `ros-humble-nav2-behaviors`
- `ros-humble-nav2-waypoint-follower`, `ros-humble-nav2-smoother`

### SLAM / Localisation
- `ros-humble-rtabmap-ros`, `ros-humble-robot-localization`

### rosidl / type-support
- `ros-humble-rosidl-typesupport-c`, `ros-humble-rosidl-typesupport-cpp`
- `ros-humble-rosidl-default-generators`, `ros-humble-rosidl-default-runtime`
- `ros-humble-rosidl-cmake`, `ros-humble-rosidl-parser`
- `ros-humble-rosidl-runtime-c`, `ros-humble-rosidl-typesupport-interface`

### System libraries
- `libopencv-dev` — OpenCV 4
- `libapriltag-dev` — AprilTag C library
- `libeigen3-dev` — Eigen 3.4
- `libglew-dev`, `libgl1-mesa-dev` — OpenGL (Pangolin dependency)
- `libboost-all-dev`, `libssl-dev`

### Built from source (inside Docker)
- **Pangolin v0.6** — installed at `/opt/pangolin`
- **ORB-SLAM3** (latest `main`) — installed at `/opt/ORB_SLAM3`
  - Includes bundled DBoW2, g2o, Sophus

---

## 14. Manual verification checklist

Run inside the container after `source install/setup.bash`:

```bash
# Package registered
ros2 pkg prefix shelfbot
# → /workspace/install/shelfbot

# Executables installed
ls /workspace/install/shelfbot/lib/shelfbot/
# → apriltag_detector_node  camera_publisher  shelfbot_slam_orb3_node
#   mission_starter.py  topic_logger.py

# Plugin XMLs installed
ls /workspace/install/shelfbot/share/shelfbot/*.xml
# → four_wheel_drive_controller_plugin.xml
#   four_wheel_drive_hardware_interface_plugin.xml

# ORB-SLAM3 library present
ls /opt/ORB_SLAM3/lib/libORB_SLAM3.so

# Pangolin library present
ls /opt/pangolin/lib/libpangolin.so

# ROS 2 packages resolvable
ros2 pkg prefix rclcpp
ros2 pkg prefix nav2_bringup
ros2 pkg prefix rtabmap_ros
ros2 pkg prefix apriltag_ros
ros2 pkg prefix cv_bridge
```
