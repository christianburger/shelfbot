# Shelfbot Project

Shelfbot is a ROS 2 application for a delivery robot, featuring autonomous navigation using the Nav2 stack and visual SLAM with RTAB-Map.

The project is a comprehensive robotic system designed to control a four-wheeled robot. It includes a detailed robot model and control interfaces that allow for real-time operation.

## Features

- **Autonomous Navigation**: Utilizes the industry-standard ROS 2 Nav2 stack for robust and flexible path planning, obstacle avoidance, and goal execution.
- **Visual SLAM**: Employs RTAB-Map for real-time Simultaneous Localization and Mapping (SLAM) using the robot's camera, allowing it to navigate in previously unseen environments.
- **Sensor Fusion**: Uses `robot_localization` (EKF) to fuse data from wheel odometry, providing a more accurate and stable estimate of the robot's position.
- **Real-World Operation**: The robot can be operated with real hardware.
- **ROS 2 Control**: Integrated with `ros2_control` for a standardized hardware abstraction layer, featuring a skid-steer drive controller.

## System Architecture

The robot's navigation capability is built on a foundation of several key ROS 2 packages that work in concert:

1.  **RTAB-Map (Visual Odometry)**: The core of the localization system. In its current configuration (RGB camera only), it functions as a Visual Odometry (VO) system. It processes sensor data (`/odom` from wheel encoders and `/camera/image_raw`) to produce a drift-corrected, smooth odometry estimate. It **does not** produce a global, fixed `map` frame.

2.  **robot_localization (EKF)**: The Extended Kalman Filter node fuses the raw wheel odometry to produce a stable `odom` -> `base_footprint` transform.

3.  **Nav2 (Navigation Stack)**: The high-level "brain" for autonomous movement. The entire stack has been configured to operate in the `odom` frame, allowing for navigation relative to the robot's starting point. Its key components include:
    *   **Planner Server**: Creates a global path from the robot's current position to the goal within the `odom` frame.
    *   **Controller Server**: Follows the global path, generating local velocity commands.
    *   **Costmaps (Global and Local)**: 2D grid representations of obstacles, populated with data from RTAB-Map's point cloud. Both are configured to operate in the `odom` frame.
    *   **BT Navigator**: Executes a Behavior Tree to manage the navigation logic.

## Launching

### Full Autonomous Navigation (Nav2 + RTAB-Map)
This is the recommended way to test the full software stack. It launches Gazebo, the robot model, RViz, RTAB-Map, and the complete Nav2 stack.

```bash
ros2 launch shelfbot nav2_dynamic.launch.py
```

### Real Robot Operation
This single launch file brings up the entire navigation stack required to operate the physical robot. It starts the hardware drivers, camera processing, `robot_localization`, RTAB-Map, and Nav2.

```bash
ros2 launch shelfbot nav2_real_robot.launch.py
```

### Visualization
To visualize the robot's state, sensor data, and navigation costmaps, use RViz. The real robot launch files will start RViz automatically. If you need to launch it separately:
```bash
ros2 launch shelfbot rviz_launch.py
```

## Node Communication

1.  The **Camera** (real) publishes images to `/camera/image_raw`.
2.  **RTAB-Map** subscribes to `/camera/image_raw` and `/odom` to generate a corrected odometry estimate and an obstacle point cloud.
3.  The **Nav2 Costmaps** subscribe to the obstacle point cloud from RTAB-Map to build their representation of the environment for planning.
4.  When a goal is sent via the **Nav2 Action Server** (`/navigate_to_pose`), the **BT Navigator** orchestrates the navigation process.
5.  The **Planner Server** creates a global plan within the `odom` frame.
6.  The **Controller Server** follows the plan, publishing velocity commands to `/cmd_vel`.
7.  The `/controller_manager` (`ros2_control`) receives these commands, sends them to the hardware interface, and reads wheel encoder data.
8.  The hardware interface reads wheel encoder positions from the hardware and passes them to the odometry calculator. The odometry calculator then computes the robot's pose and twist and publishes it to `/odom`.
9.  The `/robot_state_publisher` uses `/joint_states` to publish the robot's internal transforms (e.g., `base_link` -> `wheel_link`).
10. The `/robot_localization` node fuses the wheel odometry and publishes the final `odom` -> `base_footprint` transform.

## Using ROS 2 Command Line Tools

### Sending a Navigation Goal
To command the robot to navigate to a specific pose (e.g., x=1.0, y=0.0) relative to its starting position, you must send the goal in the `odom` frame:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Driving the Robot Manually
To drive the robot forward at 0.5 m/s, publish a `Twist` message to the controller's `cmd_vel` topic. This will override any active Nav2 goal.
```bash
ros2 topic pub /four_wheel_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

### Monitoring State
Receive state information by echoing the `joint_states` or `odom` topics:
```bash
ros2 topic echo /joint_states
ros2 topic echo /odom
```
