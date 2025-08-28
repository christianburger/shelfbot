# Agent Progress Report

This document details the progress made on the Shelfbot project's autonomous navigation capabilities.

## Phase 1: Initial State Machine (Completed & Superseded)

The initial plan was to implement a monolithic state machine in C++ to handle the mission logic.

- **Status:** A basic skeleton was created and tested.
- **Outcome:** This approach was deemed too rigid and not modular enough for future expansion. It has been entirely replaced by the Behavior Tree architecture.

## Phase 2: Refactoring to Behavior Trees (Completed)

The core of the work has been to refactor the entire mission control system to use a modern, modular Behavior Tree (BT) architecture. This provides a high-level, scriptable way to define robot behavior.

### Key Accomplishments:

1.  **Dependency Integration:**
    - The `behaviortree_cpp` library, the standard for ROS 2, has been successfully integrated into the project.
    - All necessary build system (`CMakeLists.txt`) and package (`package.xml`) changes have been made and debugged.
    - The `INSTRUCTIONS.md` has been updated to use the `rosdep` workflow for easier dependency management.

2.  **BT Engine Implementation (`mission_control_node`):**
    - The original `path_planner.cpp` has been completely rewritten to act as a BT "engine."
    - It successfully loads, parses, and "ticks" (executes) a mission defined in an external XML file.
    - All critical bugs related to shared library linking (`RPATH`), C++ smart pointers (`bad_weak_ptr`), and XML parsing have been resolved.

3.  **Creation of Reusable BT Actions (C++):**
    - A library of custom, reusable BT nodes (`shelfbot_bt_nodes`) has been created.
    - **`FindTagAction`:** A condition node that checks for the visibility of a specific AprilTag via the `/tf` tree.
    - **`MoveAction`:** A versatile action node that can execute any `Twist` command (linear, angular, or combined arc) for a specified duration. It correctly handles converting string inputs from the XML into `Twist` messages.

4.  **High-Level Mission Scripting (`mission.xml`):**
    - The mission logic has been externalized into `config/mission.xml`.
    - The current mission implements a "move-straight, turn-left, check-for-tag" patrol loop.

5.  **Advanced Maneuver Abstraction (Subtrees):**
    - The complex, multi-step "shuffle turn" required by the skid-steer robot has been encapsulated into its own reusable subtree: `config/maneuvers/turn_left_90.xml`.
    - The main mission script now calls this maneuver as a single, high-level action (`<SubTree ID="TurnLeft90"/>`), dramatically improving the readability and modularity of the main mission plan.

6.  **Simulation & Testing:**
    - A dedicated Gazebo launch file (`gazebo_mission_test.launch.py`) has been created to test the full autonomy stack (simulation + perception + mission control).
    - The system has been successfully run in simulation, demonstrating the robot executing the patrol pattern defined in the XML files.

### Current Status

The project is in a stable, working state. The Behavior Tree architecture is fully implemented and functional. The robot can successfully execute a patrol pattern composed of high-level maneuvers defined in XML. All major build system and runtime bugs encountered during this refactoring have been solved.

### Next Steps

-   Create a Gazebo world with AprilTags to allow for a full, end-to-end test of the mission (finding the origin, searching, finding the destination).
-   Implement a "GoHome" action to navigate back to a saved pose.
-   Fine-tune the parameters in the `turn_left_90.xml` maneuver for accurate turning.

## Phase 3: Real Robot Integration & Debugging (Completed)

This phase involved bringing up the full navigation stack on the real robot hardware and debugging the complex data pipeline from the hardware up to the Nav2 stack.

### Initial State: Robot Not Moving

The initial symptom was that the robot would not move when the `nav2_real_robot.launch.py` file was launched. A deep dive into the launch logs was required to identify the root cause.

### Debugging Analysis & Findings

The investigation followed the data flow from the lowest level (hardware interface) up to the highest level (VSLAM and navigation).

#### 1. Hardware Interface and Odometry: **Verified Working**
The first step was to check if the robot's hardware was communicating.
- **Evidence**: The logs clearly showed the `FourWheelDriveHardwareInterface` was receiving position data from the motor encoders and the `FourWheelDriveOdometry` node was successfully publishing the `/odom` topic.
  ```
  [ros2_control_node-2] [INFO] [1756309013.581640096] [FourWheelDriveHardwareInterface]: [2025-08-27 15:36:53] read: Read from hardware: Positions = [1.19773, 1.19773, 2.83529, 2.83529]
  [ros2_control_node-2] [INFO] [1756309013.580854244] [FourWheelDriveOdometry]: [2025-08-27 15:36:53] Update: ... Published: [X:0 Y:0 T:0]
  ```
- **Conclusion**: The control system and odometry were fully operational.

#### 2. Camera Pipeline: **Verified Working**
The next step was to verify the camera data pipeline, which involves the ESP32-CAM, the `republish` node for decompression, and the `camera_publisher` for calibration data.
- **Evidence**: The logs showed the `republish` node was actively receiving data, and the `camera_publisher` was publishing `CameraInfo` messages with valid timestamps.
  ```
  [republish-5] [DEBUG] [1756309013.112486577] [rcl]: Subscription take succeeded: true
  [camera_publisher-6] [INFO] [1756309013.136567585] [camera_info_publisher]: Publishing CameraInfo with timestamp: 1756309013.136562696
  ```
- **Conclusion**: The `/camera/image_raw` and `/camera/camera_info` topics were being published correctly.

#### 3. RTAB-Map VSLAM: **Identified as Root Cause**
With all input topics confirmed to be working, the focus shifted to the `rtabmap` node itself.
- **Symptom**: The node consistently produced a "Did not receive data" warning, indicating it was not processing the inputs.
- **Root Cause Analysis**: A detailed inspection of the node's startup logs revealed a critical command-line parsing error.
  ```
  [rtabmap-8] [DEBUG] [1756309013.600179631] [rcl]: Couldn't parse arg 10 (-d) as a remap rule in its deprecated form. Error: Expecting token or wildcard, at ./src/rcl/arguments.c:1170
  ```
  This error, though only at the DEBUG level, was a symptom of a deeper issue. The ROS 2 `rcl` parser was getting confused by the mix of application-specific arguments (like `-d` for "delete database") and ROS-specific arguments (like `-r` for remapping) in the launch file. This confusion caused the node to **fail to apply its topic remappings**, even though the log showed them being parsed. As a result, the node was listening on its default topic names (`rgb/image`, `rgb/camera_info`) instead of the correct ones (`/camera/image_raw`, `/camera/camera_info`), causing the timeout.

### The Solution

The definitive fix was to restructure the `rtabmap_node` definition in `nav2_real_robot.launch.py` to be unambiguous to the `rcl` parser. This involved:
1.  Moving all ROS-specific arguments (logging and remappings) to come first, immediately after the `--ros-args` flag.
2.  Using the `--` separator to explicitly tell the `rcl` parser to stop processing arguments.
3.  Placing all application-specific arguments (`-d`) after the `--` separator.

This corrected code ensures the parser correctly applies the remappings, allowing `rtabmap` to subscribe to the proper topics and begin the SLAM process. With this fix, the entire data pipeline is now operational.
