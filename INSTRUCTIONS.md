# Shelfbot Setup Instructions

This document provides detailed instructions for setting up the required dependencies to build and run the Shelfbot project.

## 1. Automated Dependency Installation (Recommended)

The recommended method for installing all required system dependencies is to use `rosdep`. This tool reads the `package.xml` file and automatically installs the correct versions of all required libraries for your specific OS and ROS 2 distribution.

Run the following commands from the root of your workspace (e.g., `~/shelfbot_workspace`):

```bash
# First, update the rosdep sources to ensure you have the latest package list
rosdep update

# Next, run the installer. This command finds all dependencies in the 'src'
# directory, ignores packages that are already in your source tree,
# and installs the rest.
rosdep install --from-paths src --ignore-src -r -y
```

## 2. List of Project Dependencies

The `rosdep` command will install the ROS 2 system equivalents of the following packages, which are declared in `package.xml`:

#### Core
- `rclcpp`
- `sensor_msgs`
- `tf2_ros`
- `rviz2`
- `gazebo_ros`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `controller_interface`
- `hardware_interface`
- `realtime_tools`
- `pluginlib`
- `gazebo_ros2_control`
- `xacro`
- `ros2_controllers`

#### Perception
- `apriltag_ros`
- `apriltag_msgs`
- `image_transport`
- `cv_bridge`
- `visualization_msgs`

#### Navigation & Mapping
- `nav2_bringup`
- `nav2_bt_navigator`
- `nav2_controller`
- `nav2_planner`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `nav2_common`
- `rtabmap_ros`
- `robot_localization`
- `camera_info_manager`


## 3. Manual Verification (Optional)

After running `rosdep`, you can optionally run the following commands to confirm that all packages were installed correctly. Each command should return a path (e.g., `/opt/ros/humble/share/rclcpp`). If a command returns nothing, the package is missing.

```bash
# Core
ros2 pkg prefix rclcpp
ros2 pkg prefix sensor_msgs
ros2 pkg prefix tf2_ros
ros2 pkg prefix rviz2
ros2 pkg prefix gazebo_ros
ros2 pkg prefix geometry_msgs
ros2 pkg prefix nav_msgs
ros2 pkg prefix tf2
ros2 pkg prefix controller_interface
ros2 pkg prefix hardware_interface
ros2 pkg prefix realtime_tools
ros2 pkg prefix pluginlib
ros2 pkg prefix gazebo_ros2_control
ros2 pkg prefix xacro
ros2 pkg prefix ros2_controllers

# Perception
ros2 pkg prefix apriltag_ros
ros2 pkg prefix apriltag_msgs
ros2 pkg prefix image_transport
ros2 pkg prefix cv_bridge
ros2 pkg prefix visualization_msgs

# Navigation & Mapping
ros2 pkg prefix nav2_bringup
ros2 pkg prefix rtabmap_ros
ros2 pkg prefix robot_localization
ros2 pkg prefix camera_info_manager
```