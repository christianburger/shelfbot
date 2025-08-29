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

## Current Status & Next Steps

- **SUCCESS:** The "Goal was rejected" error is **RESOLVED**. The system now correctly accepts navigation goals sent in the `odom` frame.
- **NEW PROBLEM:** The robot **does not move** after accepting the goal.

### Plan for Tomorrow:
The debugging process will continue by investigating the data flow *after* the goal is accepted.
1.  **Check `bt_navigator`:** Verify that the behavior tree is running and is sending commands to the `controller_server`.
2.  **Check `controller_server`:** Verify that it is receiving commands and publishing velocity commands to `/cmd_vel`.
3.  **Check `ros2_control`:** Verify that the `four_wheel_drive_controller` is receiving `/cmd_vel` messages.
4.  **Check Hardware Interface:** Verify that the `write()` method in the `FourWheelDriveHardwareInterface` is being called with non-zero velocity commands.
5.  **Check Micro-ROS Agent:** Verify that the commands are being successfully transmitted to the ESP32 firmware.
