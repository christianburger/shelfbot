# Shelfbot Progress Report - End of Day

This document details the progress made on the Shelfbot project's autonomous navigation capabilities, summarizing a long and intensive debugging session.

## Initial State & Core Problem

The day began with a critical failure: the Nav2 stack was consistently rejecting all navigation goals with the message "Goal was rejected." The system was launching without fatal errors, and all action servers were available, but the robot was completely non-functional.

## Debugging Journey & Discoveries

The investigation was a process of peeling back layers of a complex, cascading failure.

### Phase 1: Chasing Symptoms (Incorrect Theories)
- **TF Mismatches:** Early efforts focused on TF frame ID mismatches (`base_link` vs. `base_footprint`) and TF timing issues (`extrapolation` errors). While these were real problems that were fixed, they were not the root cause.
- **Costmap Configuration:** The investigation then moved to the costmaps, which were failing to load. This was a critical issue, but fixing it (by correcting the parameter namespacing in `nav2_camera_params.yaml`) still did not resolve the goal rejection.
- **Hardware Interface & `ros2_control`:** A deep dive into the hardware interface and `ros2_control` configuration was performed. While some minor bugs were fixed, this was also not the root cause.

### Phase 2: The Breakthrough - Identifying the Real Root Cause
- **The `bt_navigator` Failure:** The turning point came when a detailed inspection of the `launch.log`, guided by the user, revealed that the `bt_navigator` node was failing to activate.
- **Missing Plugins:** The logs showed a series of "Node not recognized" errors (`RecoveryNode`, `PipelineSequence`, `RateController`, etc.). This proved that the `bt_navigator` was failing because it could not find the C++ plugins for the nodes specified in the behavior tree XML file.
- **The Fix:** The solution was to systematically add each missing BT node plugin to the `plugin_lib_names` list in `nav2_camera_params.yaml`.

### Phase 3: Architectural Correction - `map` vs. `odom`
- **The Final Hurdle:** Even after fixing the `bt_navigator`, goals were still being rejected.
- **The Root Architectural Flaw:** A research phase, prompted by the user, revealed the fundamental misunderstanding: our system, with an RGB-only camera, can only support **Visual Odometry** (providing an `odom` frame), not full SLAM (which would provide a `map` frame).
- **The Fix:** The entire Nav2 stack was reconfigured to operate in the `odom` frame. This involved changing the `global_frame` parameter in the costmaps, behavior server, and `bt_navigator` sections of `nav2_camera_params.yaml`.

## Phase 4: From Build Success to Execution Failure

Following the major architectural changes, the immediate goal was to achieve a clean build and assess the new baseline.

### Build System Cleanup
The project restructuring introduced a series of build errors which were systematically resolved:
- **Missing `find_package` calls:** Added to all component `CMakeLists.txt` files to ensure dependencies like `rclcpp`, `realtime_tools`, and `std_msgs` were found before being used.
- **Redundant `ament_package()` calls:** Removed from all subdirectories, leaving only the single required call in the root `CMakeLists.txt` to fix package generation conflicts.
- **Incorrect Include Paths:** Corrected numerous `#include` statements where a global search-and-replace had incorrectly prefixed system headers (e.g., `rclcpp/rclcpp.hpp`) with `shelfbot/`.

### Runtime Analysis: A New Root Cause

After achieving a successful compilation, a new launch revealed a clearer, more fundamental issue.

- **Initial Crash:** The `ros2_control_node` immediately crashed with a `pluginlib::LibraryLoadException`, as it could not find the `FourWheelDriveHardwareInterface`.
- **The Fix:** The root `CMakeLists.txt` was missing the necessary `install(TARGETS ...)` command for the hardware interface library and the `pluginlib_export_plugin_description_file` call to make it visible to the ROS 2 ecosystem. Correcting this resolved the crash.

With `ros2_control` stable, the system came online, but navigation goals still failed. A detailed log analysis for goal ID `8c06e935e48c40be8a0abf99391a4b48` revealed the precise data flow and failure point.

#### Data Flow & Failure Chain (Goal ID: 8c06e935e48c40be8a0abf99391a4b48)

1.  **`[1756484006.308]` `[bt_navigator]`**: Goal is accepted and navigation begins.
2.  **`[~1756484007.0]` `[planner_server]`**: Receives the request and successfully computes a valid path from the robot's location to the goal.
3.  **`[1756484322.165]` `[controller_server]`**: Receives the new path from the `bt_navigator` to begin execution.
4.  **`[1756484322.287]` `[controller_server]`**: **FAILURE.** The `nav2_costmap_2d` component immediately logs `Robot is out of bounds of the costmap!`.
5.  **`[1756484194.170]` `[bt_navigator]`**: After the controller repeatedly fails to make progress, the behavior tree aborts all retries and marks the goal as failed.

---

## Current Status & Next Steps

### Component Health Checklist

-   **Build System:** ✅ **Healthy**. The project now compiles without errors.
-   **ros2_control (`ros2_control_node`)**: ✅ **Healthy**. The node starts, correctly loads the hardware interface plugin, and communicates with the hardware.
-   **Odometry (`four_wheel_drive_odometry`)**: ✅ **Healthy**. The `/odom` topic is being published at a steady rate, and the `odom` -> `base_footprint` transform is correct.
-   **RTAB-Map (`rtabmap`)**: ⚠️ **Degraded**. The node is running but is continuously warning about TF extrapolation errors (`Lookup would require extrapolation into the future`). This indicates a timing problem that is preventing it from reliably creating a map, which is the likely source of the missing `map` frame.
-   **Nav2 Lifecycle Manager**: ✅ **Healthy**. All Nav2 nodes are brought up and transitioned to the `active` state correctly.
-   **Nav2 Planner (`planner_server`)**: ✅ **Healthy**. It is capable of generating paths when the TF tree is valid.
-   **Nav2 Controller (`controller_server`)**: ❌ **Unhealthy**. The node is active, but its local costmap configuration is incorrect, causing it to believe the robot is always outside of its operational area. This is the **immediate blocker** for any robot motion.

### Plan Forward

The investigation has successfully pinpointed the problem. The next steps are clear and targeted:

1.  **Primary Target: Fix the Costmap Boundary.** The `Robot is out of bounds of the costmap!` error must be resolved. This will be done by adjusting the `local_costmap` parameters in `nav2_camera_params.yaml`. The `width` and `height` of the costmap likely need to be increased to give the robot more space.
2.  **Secondary Target: Stabilize RTAB-Map.** Address the TF `extrapolation` warnings to ensure the `map` -> `odom` transform is published reliably. This may involve tuning `tf_delay` or QoS settings for the odometry and camera topics.
