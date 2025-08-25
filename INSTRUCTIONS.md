# Shelfbot Setup Instructions

This document provides detailed instructions for setting up the required dependencies to build and run the Shelfbot project.

## System Dependencies Verification

Before building the workspace, you must ensure that all required ROS 2 packages are installed.

Run the verification script below in your terminal. Each command should return a path (e.g., `/opt/ros/humble`). If any command reports `Package not found`, you must install the corresponding package using the `apt` instructions provided in the next section.

### Verification Script

```bash
#!/bin/bash
PACKAGES=(
    # Core
    rclcpp
    sensor_msgs
    tf2_ros
    rviz2
    gazebo_ros
    geometry_msgs
    nav_msgs
    tf2
    controller_interface
    hardware_interface
    realtime_tools
    pluginlib
    gazebo_ros2_control
    xacro
    ros2_controllers
    # Perception
    apriltag
    apriltag_ros
    apriltag_msgs
    image_transport
    cv_bridge
    visualization_msgs
)

for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg prefix "$pkg" > /dev/null 2>&1; then
        echo "✅ $pkg found"
    else
        echo "❌ $pkg NOT found"
    fi
done
```

## Installation Commands

If any package is missing, install it using the appropriate command from the list below.

### Core Dependencies
```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-tf2-ros \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2 \
  ros-humble-controller-interface \
  ros-humble-hardware-interface \
  ros-humble-realtime-tools \
  ros-humble-pluginlib \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-ros2-controllers
```

### Perception Dependencies
```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-apriltag \
  ros-humble-apriltag-ros \
  ros-humble-apriltag-msgs \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs
```
