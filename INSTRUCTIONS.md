# Shelfbot Setup Instructions

This document covers the complete setup of the Shelfbot project using Docker.
Docker is the **recommended approach**: it gives you a reproducible ROS 2 Humble
environment on any Linux host without polluting your system packages.

---

## Prerequisites

| Requirement | Minimum version |
|---|---|
| Docker Engine | 24.x |
| Docker Compose plugin | 2.x (`docker compose` — note: no hyphen) |
| NVIDIA driver (optional) | 525+ if you need GPU-accelerated OpenCV / ORB-SLAM3 |

Install Docker on Ubuntu:
```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # log out and back in after this
```

---

## 1. Repository layout expected on the host

```
shelfbot_project/          ← clone / place this repo here
├── Dockerfile
├── docker-compose.yml
├── package.xml
├── CMakeLists.txt
├── ...all other source files...
└── shelfbot_ws/           ← created automatically; two-way shared volume
    └── src/
        └── shelfbot/      ← symlink or copy of the package source
```

Create the workspace source directory and symlink the package into it:
```bash
mkdir -p shelfbot_ws/src
ln -s "$(pwd)" shelfbot_ws/src/shelfbot
```

---

## 2. Building the Docker image

### 2a. Full build (includes ORB-SLAM3)

> ⚠️ **This takes 20–40 minutes** the first time because it compiles
> Pangolin and ORB-SLAM3 from source inside the image.

```bash
docker compose build
```

### 2b. Development build (skip ORB-SLAM3)

If you are not using the ORB-SLAM3 SLAM back-end, uncomment the
`target: base` line in `docker-compose.yml` before building:

```yaml
build:
  context: .
  dockerfile: Dockerfile
  target: base    # ← uncomment this
```

Then build as normal:
```bash
docker compose build
```

---

## 3. Starting the container

```bash
# Allow the container to draw on your X11 display (GUI apps)
xhost +local:docker

# Start the container in the background
docker compose up -d
```

The container starts an interactive bash shell by default. The `shelfbot_ws/`
directory on your host is mounted read-write at `/workspace` inside the
container — any file you edit on either side is immediately visible on the other.

---

## 4. Opening a shell inside the container

```bash
# Primary shell
docker compose exec shelfbot bash

# Additional shells (run this in a new terminal)
docker compose exec shelfbot bash
```

You can open as many shells as you need simultaneously.

---

## 5. Building the ROS 2 package inside the container

Run these commands **inside the container** (after step 4):

```bash
cd /workspace

# Install any remaining rosdep dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

Add the source line to your shell profile inside the container if you want
it to persist for new shells:
```bash
echo "source /workspace/install/setup.bash" >> ~/.bashrc
```

---

## 6. Launching the robot

All launch files are available once the workspace is built.

```bash
# Real robot with micro-ROS bridge
ros2 launch shelfbot real_robot_microros.launch.py

# Real robot + RTAB-Map SLAM + Nav2
ros2 launch shelfbot nav2_bt_real_robot.launch.py

# ORB-SLAM3 + Nav2 (requires full build with ORB-SLAM3)
ros2 launch shelfbot nav2_orb_slam3.launch.py
```

---

## 7. Stopping and re-entering

```bash
# Stop the container (data in /workspace is preserved on the host)
docker compose down

# Start again
docker compose up -d
docker compose exec shelfbot bash
```

---

## 8. Removing the image to rebuild from scratch

```bash
docker compose down --rmi all --volumes
docker compose build
```

---

## 9. Verified ROS 2 Humble dependency list

The following packages are installed in the Docker image.
All names have been verified against the Ubuntu 22.04 (jammy) APT repository
for ROS 2 Humble. **None of these are ROS 1 packages.**

### Core rclcpp
- `ros-humble-rclcpp`
- `ros-humble-rclcpp-lifecycle`

### Common message packages
- `ros-humble-std-msgs`
- `ros-humble-sensor-msgs`
- `ros-humble-geometry-msgs`
- `ros-humble-nav-msgs`
- `ros-humble-visualization-msgs`

### TF2
- `ros-humble-tf2`
- `ros-humble-tf2-ros`
- `ros-humble-tf2-geometry-msgs`

### ros2_control
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-controller-interface`
- `ros-humble-hardware-interface`
- `ros-humble-realtime-tools`
- `ros-humble-pluginlib`

### Robot description
- `ros-humble-xacro`
- `ros-humble-robot-state-publisher`
- `ros-humble-joint-state-publisher`
- `ros-humble-joint-state-publisher-gui`

### Visualisation
- `ros-humble-rviz2`

### Vision / Perception
- `ros-humble-image-transport`
- `ros-humble-image-transport-plugins`
- `ros-humble-cv-bridge`
- `ros-humble-camera-info-manager`
- `ros-humble-message-filters`
- `ros-humble-apriltag-ros`
- `ros-humble-apriltag-msgs`

### Navigation 2
- `ros-humble-nav2-bringup`
- `ros-humble-nav2-controller`
- `ros-humble-nav2-planner`
- `ros-humble-nav2-map-server`
- `ros-humble-nav2-lifecycle-manager`
- `ros-humble-nav2-common`
- `ros-humble-nav2-bt-navigator`
- `ros-humble-nav2-behaviors`
- `ros-humble-nav2-waypoint-follower`
- `ros-humble-nav2-smoother`

### SLAM / Localisation
- `ros-humble-rtabmap-ros`
- `ros-humble-robot-localization`

### System (non-ROS) libraries
- `libopencv-dev` — OpenCV
- `libapriltag-dev` — AprilTag C library (used directly)
- `libeigen3-dev` — Linear algebra
- `libglew-dev` — OpenGL extension wrangler (Pangolin dependency)
- `libgl1-mesa-dev` — OpenGL (Pangolin dependency)

### Built from source (inside Docker)
- **Pangolin v0.6** — 3D visualisation for ORB-SLAM3
- **ORB-SLAM3** — Monocular SLAM; installed at `/opt/ORB_SLAM3`

---

## 10. Packages intentionally excluded

| Package | Reason |
|---|---|
| `gazebo_ros` / `gazebo_ros2_control` | No Gazebo simulation code in this project |
| `behaviortree_cpp` | Not linked in any CMakeLists; use `behaviortree_cpp_v3` if you add BT nodes |

---

## 11. Manual verification (inside container)

After the image is built you can confirm each package is present:

```bash
# ROS 2 packages
ros2 pkg prefix rclcpp
ros2 pkg prefix sensor_msgs
ros2 pkg prefix tf2_ros
ros2 pkg prefix rviz2
ros2 pkg prefix nav2_bringup
ros2 pkg prefix rtabmap_ros
ros2 pkg prefix robot_localization
ros2 pkg prefix apriltag_ros
ros2 pkg prefix camera_info_manager
ros2 pkg prefix image_transport
ros2 pkg prefix cv_bridge

# ORB-SLAM3
ls /opt/ORB_SLAM3/lib/libORB_SLAM3.so

# Pangolin
ls /opt/pangolin/lib/libpangolin.so
```

Each ROS 2 command should return a path such as
`/opt/ros/humble/share/<package>`. If a command returns nothing the
package is missing and `docker compose build` should be re-run after
checking the Dockerfile.
