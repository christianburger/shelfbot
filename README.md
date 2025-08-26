# Shelfbot Project

Shelfbot is a ROS2 application of a delivery robot.

The Shelfbot project is a comprehensive robotic system designed to simulate and control a four-wheeled robot. This project includes a detailed robot model, simulation environments, and control interfaces that allow for both simulated and real-time operation. The Shelfbot is equipped with a variety of components, including wheels, motors, and sensors, and is capable of performing complex maneuvers and tasks.

## Autonomous Navigation with Behavior Trees

High-level mission logic for the Shelfbot is defined using **Behavior Trees (BTs)**. This is a modern, modular, and flexible approach to robotics task management that is used by ROS 2's own Nav2 stack. It replaces a monolithic, hardcoded state machine with a "scriptable" system that is easy to read and modify.

### How it Works

The system is composed of three main parts:

1.  **The Mission Plan (`mission.xml`):** The "script" that defines the robot's goals. This is a simple XML file located in `/config/mission.xml`. It describes the mission as a tree of nodes, where each node represents a goal or a logical operator.

2.  **The BT Engine (`mission_control_node`):** This is the C++ "brain" of the robot. It reads the `mission.xml` file, and at a regular interval (10 Hz), it "ticks" the tree. Ticking the tree executes the logic defined in the XML, causing the robot to perform actions.

3.  **The Actions & Conditions (C++ Classes):** These are the building blocks of the tree. Each C++ class is a small, self-contained, and reusable piece of code that performs a single task. For example:
    *   `FindTagAction`: Checks if a specific AprilTag is visible to the robot.
    *   `MoveAction`: Commands the robot to move with a specific velocity for a set duration.

### Translating the Behavior Tree into Actions

The `mission_control_node` translates the XML script into robot behavior through the following data flow:

1.  **Tree Execution:** The BT engine ticks the tree. Let's say it's currently on a `<MoveAction>` node in the XML.
2.  **Action Invocation:** The engine invokes the corresponding `MoveAction` C++ class.
3.  **ROS 2 Communication:** The `MoveAction` class publishes a `geometry_msgs/msg/Twist` message to the `/four_wheel_drive_controller/cmd_vel` topic.
4.  **Robot Movement:** The `ros2_control` system receives this message and commands the motors to move the robot, as described in the "Node Communication" section.

### Role of AprilTags

The AprilTags are the primary landmarks for navigation. The Behavior Tree uses them to make decisions:

1.  The robot's camera publishes images.
2.  The `/apriltag_detector_node` processes these images and publishes the 3D pose of any visible tags to the `/tf` topic.
3.  The `FindTagAction` C++ class uses a TF2 listener to check if a specific tag's transform (e.g., `tag_5`) exists in the `/tf` tree.
4.  If the transform exists, the `FindTagAction` returns `SUCCESS` to the Behavior Tree, which can then transition to the next step in the mission (e.g., stop searching and return home).

### The Implemented Mission (`mission.xml`)

The current mission implements a continuous search pattern. The logic can be read from top to bottom in the XML file:

1.  **`SetBlackboard`:** First, a command is created to define an "arc move" (moving forward at 0.2 m/s while turning at 0.4 rad/s). This command is stored in a variable called `search_twist` on the BT's "Blackboard" (a shared memory space).

2.  **`KeepRunningUntilFailure` (The Main Loop):** The tree enters a loop that will repeat forever until one of its children *fails*.

3.  **`Sequence` (The Patrol Step):** Inside the loop, a sequence is executed:
    a.  **`MoveAction`:** The robot executes the `search_twist` command for 1.0 second, causing it to move in an arc.
    b.  **`Inverter` & `FindTag`:** Immediately after, it checks for the destination tag (ID 5). The `Inverter` flips the logic:
        *   If the tag is **not** found, `FindTag` returns `FAILURE`, which the `Inverter` turns into `SUCCESS`. The sequence succeeds, and the main loop runs the patrol step again.
        *   If the tag **is** found, `FindTag` returns `SUCCESS`, which the `Inverter` turns into `FAILURE`. The sequence fails, causing the main `KeepRunningUntilFailure` loop to stop.

4.  **`MoveAction` (Stop):** Once the loop is broken (meaning the tag was found), a final `MoveAction` is called to send a zero-velocity command, stopping the robot.

This creates a robust, non-blocking search where the robot is continuously moving and checking for the tag on every cycle.

## Capabilities

- **Simulation and Real-Time Control**: The Shelfbot can be operated in both simulated environments (Gazebo and Isaac Sim) and real-time scenarios.
- **Four-Wheel Drive System**: The robot is equipped with a four-wheel drive system, allowing for precise control and movement.
- **Sensor Integration**: The model includes sensors for navigation and environment interaction.
- **ROS 2 Integration**: The project is built on ROS 2, providing robust communication and control capabilities.

## Robot Model

The robot model is defined using URDF and Xacro files, which describe the physical and visual properties of the robot. Key components include:

- **Wheels**: Modeled using `wheel.xacro`, defining the size and material properties.
- **Motors**: Defined in `motor.xacro`, specifying the motor dimensions and inertial properties.
- **Base Axes**: Described in `base_axis.xacro`, detailing the rotational joints for wheel movement.
- **Camera**: Integrated using `camera.xacro`, providing visual feedback for navigation.
- **Common Properties**: Shared properties and materials are defined in `common_properties.xacro`.

## Hardware Layout

### Motor Wiring Order

The physical motors are wired to the ESP32 controller in a specific, non-sequential order. The mapping from the motor's physical location on the robot to its index in the firmware (`FastAccelStepper` array) is as follows:

| Motor Index (Firmware) | Physical Location |
| :--------------------- | :---------------- |
| 0                      | Front Left        |
| 1                      | Back Left         |
| 2                      | Back Right        |
| 3                      | Front Right       |

This order is critical and is reflected in the ROS 2 controller configuration.

## Launching the Simulation

To launch the Shelfbot in a simulation environment, use the provided launch files:

Gazebo Simulation: Launch the robot in Gazebo using: 
  ros2 launch shelfbot gazebo_sim.launch.py

Isaac Sim: Launch the robot in Isaac Sim using: 
  ros2 launch shelfbot isaac_sim_shelfbot_launch.py

RViz Visualization: Visualize the robot in RViz using: 
  ros2 launch shelfbot rviz_launch.py

Note: The Gazebo and Isaac Sim launchers automatically start RViz.

## Node Communication

This section details the communication architecture of the Shelfbot system, outlining the key nodes and the topics they use to interact.

### `/apriltag_detector_node`
The `apriltag_detector_node` is responsible for detecting Apriltags in the environment.

- **Subscribers**:
  - `/camera/image_raw` (`sensor_msgs/msg/Image`): Consumes the raw image feed from the camera.
  - `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`): Consumes the camera's calibration data.
- **Publishers**:
  - `/tag_poses` (`geometry_msgs/msg/PoseArray`): Publishes the 3D poses of all detected Apriltags.
  - `/tag_markers` (`visualization_msgs/msg/MarkerArray`): Publishes visual markers for RViz to display the detected tags.
  - `/tf` (`tf2_msgs/msg/TFMessage`): Broadcasts the live transform for each detected tag relative to the camera.

### `/controller_manager` (ros2_control)
This is the main node from the `ros2_control` framework that manages the hardware interfaces and controllers.

- **Subscribers**:
  - `/four_wheel_drive_controller/cmd_vel` (`geometry_msgs/msg/Twist`): Receives velocity commands to drive the robot. This is the primary control topic for navigation.
  - `/four_wheel_drive_controller/direct_commands` (`std_msgs/msg/Float64MultiArray`): Receives direct velocity commands for each wheel, typically used for testing.
- **Publishers**:
  - `/joint_states` (`sensor_msgs/msg/JointState`): Publishes the current state (position, velocity) of all robot joints.
  - `/odom` (`nav_msgs/msg/Odometry`): Publishes the robot's estimated odometry based on wheel encoder feedback.
  - `/tf` (`tf2_msgs/msg/TFMessage`): Broadcasts the `odom` -> `base_footprint` transform.

### `/robot_state_publisher`
This standard ROS 2 node uses the robot's URDF and the `/joint_states` topic to compute and publish the transforms for all the fixed and moving parts of the robot.

- **Subscribers**:
  - `/robot_description` (`std_msgs/msg/String`): Reads the URDF model of the robot on startup.
  - `/joint_states` (`sensor_msgs/msg/JointState`): Consumes the joint states to calculate the transforms for moving parts (like wheels).
- **Publishers**:
  - `/tf` (`tf2_msgs/msg/TFMessage`): Broadcasts the transforms for all links in the robot model (e.g., `base_link` -> `wheel_link`).

### Data Flow and Dependencies
1.  The **Camera** (either simulated in Gazebo or a real hardware driver) publishes images to `/camera/image_raw`.
2.  The `/apriltag_detector_node` subscribes to these images, detects tags, and publishes their 3D poses to `/tf` and `/tag_poses`.
3.  A high-level navigation stack (like Nav2, not yet implemented) would consume the `/tf` data and the `/odom` data to determine the robot's position and issue velocity commands.
4.  The navigation stack would publish these commands to `/four_wheel_drive_controller/cmd_vel`.
5.  The `/controller_manager` receives these commands and passes them to the `FourWheelDriveHardwareInterface`, which sends the appropriate signals to the motors (either in simulation or via the Micro-ROS agent on the real robot).
6.  The hardware interface reads the wheel encoders and publishes the joint states to `/joint_states`.
7.  The `/robot_state_publisher` consumes `/joint_states` to publish the robot's internal transforms to `/tf`.

## Hardware Interface

The Shelfbot includes a hardware interface for controlling and monitoring the robot's state:

Command Interfaces: Allow sending commands to the robot's joints and actuators.
State Interfaces: Provide feedback on the robot's current state, including joint positions and velocities.

## Using ROS 2 Command Line Tools

To interact with the hardware interface, you can use ROS 2 command line tools.

### Driving the Robot

To drive the robot forward at 0.5 m/s, publish a `Twist` message to the controller's `cmd_vel` topic:

  ros2 topic pub /four_wheel_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1

### Direct Wheel Control

To command the individual wheel joints directly (e.g., for testing), publish to the `direct_commands` topic:

  ros2 topic pub /four_wheel_drive_controller/direct_commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]"

### Monitoring State

Receive state information by echoing the `joint_states` topic:

  ros2 topic echo /joint_states

## Performance Considerations

The robot uses a skid-steer drive mechanism. Due to the physics of this design, turning maneuvers on high-friction surfaces (like concrete or carpet) can cause significant resistance. This requires high torque from the motors and can lead to jittering, sliding, or wheels struggling to turn, especially during zero-radius (in-place) pivots.

The robot's heavy mass and direct-drive gearing amplify this issue. For best performance, it is recommended to command **arc turns** (where the robot moves forward or backward while turning) rather than stationary pivots. This can be achieved by always providing a non-zero linear velocity (`linear.x`) in the `Twist` command when an angular velocity (`angular.z`) is present.

## Conclusion

The Shelfbot project provides a versatile platform for simulating and controlling a four-wheeled robot.
With its integration into ROS 2, it offers a robust framework for developing and testing robotic applications in both simulated and real-world environments.
