# Shelfbot

Shelfbot is a ROS 2 Humble application for a four-wheeled skid-steer delivery robot.
It provides autonomous navigation (Nav2), visual SLAM (RTAB-Map or ORB-SLAM3),
hardware abstraction via `ros2_control`, AprilTag detection, and a micro-ROS bridge
to ESP32 firmware.

**Development environment:** Docker is the recommended approach.
See [`INSTRUCTIONS.md`](INSTRUCTIONS.md) for the complete Docker setup.
See [`ENVIRONMENT.md`](ENVIRONMENT.md) for native (bare-metal) installation.

---

## Repository layout

```
/home/chris/shelfbot/shelfbot/     ← repo root  (this directory)
├── CMakeLists.txt
├── CMakePresets.json              ← CLion CMake profiles
├── package.xml
├── Dockerfile
├── docker-compose.yml
├── docker-entrypoint.sh           ← auto-fixes /workspace ownership on start
├── .env                           ← DOCKER_UID / DOCKER_GID (1000/1000)
├── .devcontainer/devcontainer.json
├── include/shelfbot/
├── src/
│   ├── control/                   ← FourWheelDriveController, HardwareInterface
│   ├── hardware/                  ← MicroRosCommunication
│   ├── navigation/                ← FourWheelDriveOdometry
│   ├── nodes/                     ← camera_publisher, apriltag_detector, slam_orb3
│   ├── perception/
│   └── utils/                     ← shelfbot_utils (logging)
├── config/
├── launch/
├── urdf/
├── scripts/
└── shelfbot_ws/                   ← colcon workspace (bind-mounted to /workspace)
    ├── src/shelfbot → repo root   ← provided by second Docker bind-mount
    ├── build/
    ├── install/
    └── log/
```

---

## Features

- **Autonomous Navigation** — Nav2 stack (planner, controller, costmaps, BT navigator)
- **Visual SLAM** — RTAB-Map (production) or ORB-SLAM3 (optional, baked into Docker image)
- **Sensor Fusion** — `robot_localization` EKF fuses wheel odometry
- **ros2_control** — standardised hardware abstraction; four-wheel skid-steer controller
- **AprilTag Detection** — pose estimation via the apriltag C library; TF + marker publishing
- **micro-ROS Bridge** — talks to ESP32 firmware over serial or UDP
- **Docker Environment** — reproducible ROS 2 Humble build; ORB-SLAM3 + Pangolin compiled inside image

---

## Quick start (Docker)

```bash
# 1. Clone and enter repo
cd /home/chris/shelfbot/shelfbot

# 2. Create the colcon workspace directory as chris (must exist before docker compose up)
mkdir -p shelfbot_ws

# 3. Allow GUI apps (RViz2) to use the host display
xhost +local:docker

# 4. Build the image (~10 min base, ~20 min full with ORB-SLAM3)
docker compose build

# 5. Start
docker compose up -d

# 6. Open a shell (runs as chris, /workspace is writable)
docker compose exec shelfbot bash
```

Inside the container:

```bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
ros2 pkg prefix shelfbot    # → /workspace/install/shelfbot
```

---

## System architecture

```
ESP32 firmware
    │  serial/UDP (micro-ROS)
    ▼
micro_ros_agent  ──►  /shelfbot_firmware/motor_positions
                       /shelfbot_firmware/set_speed
                       /shelfbot_firmware/motor_command
    │
    ▼
FourWheelDriveHardwareInterface  (ros2_control plugin)
    │  hw_positions[]
    ▼
FourWheelDriveOdometry  ──►  /odom  +  TF: odom → base_footprint
    │
    ▼
robot_localization (EKF)  ──►  TF: odom → base_footprint (fused)
    │
    ├──► RTAB-Map  ──►  /map  +  TF: map → odom  +  point cloud for costmaps
    │    (or ORB-SLAM3)
    │
    ▼
Nav2 stack
    ├── planner_server    → /plan
    ├── controller_server → /cmd_vel
    ├── bt_navigator      → /navigate_to_pose action
    └── costmap servers   ← RTAB-Map point cloud

FourWheelDriveController  (ros2_control plugin)
    ◄── /four_wheel_drive_controller/cmd_vel  (from Nav2 or manual)

camera_publisher  ──►  /camera/image_raw  +  /camera/camera_info
apriltag_detector ──►  /tag_poses  +  /apriltag_markers  +  TF: camera_link → tag_N
```

---

## Launch options

All commands run inside the container after `source install/setup.bash`.

| Goal | Command |
|---|---|
| Full stack: Nav2 + RTAB-Map | `ros2 launch shelfbot nav2_bt_real_robot.launch.py` |
| Nav2 + ORB-SLAM3 | `ros2 launch shelfbot nav2_orb_slam3.launch.py` |
| Nav2, odometry only (no SLAM) | `ros2 launch shelfbot nav2_real_robot.launch.py` |
| AprilTag detector only | `ros2 launch shelfbot apriltag_detector.launch.py tag_size:=0.16` |

Three terminals are required — see [`INSTRUCTIONS.md § Launch`](INSTRUCTIONS.md#6-launching-the-robot).

---

## Key ROS 2 commands

```bash
# Manual velocity (robot will move — caution)
ros2 topic pub --once /four_wheel_drive_controller/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Navigation goal in odom frame
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, \
    pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Health checks
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic hz /shelfbot_firmware/motor_positions
ros2 run tf2_tools view_frames
```

---

## Known build warnings

When compiling `shelfbot_slam_orb3_node`, GCC emits multiple
`Eigen::AlignedBit is deprecated [-Wdeprecated-declarations]` warnings.
These originate in ORB-SLAM3's bundled g2o third-party library (which targets
Eigen 3.3) and are **harmless** — the build and resulting binary are correct.
Ubuntu 22.04 ships Eigen 3.4, which deprecated `AlignedBit`.
The warnings cannot be fixed without patching upstream g2o.
They are suppressed in `src/nodes/CMakeLists.txt` via
`target_compile_options(shelfbot_slam_orb3_node PRIVATE -Wno-deprecated-declarations)`.
