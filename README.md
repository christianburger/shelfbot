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

The current mission demonstrates a robust, multi-step autonomous navigation task using Nav2. The logic can be read from top to bottom in the `/config/mission.xml` file:

1.  **`SetBlackboard` (Define Patrol Route):** An array of four poses is defined and stored on the Behavior Tree's "Blackboard" (a shared memory space) under the key `patrol_route`. This defines a square patrol pattern.

2.  **`NavigateToPose` (Move to Start):** Before starting the main search, the robot navigates to the first waypoint of the patrol route. This ensures the robot is in a known state and prevents it from immediately finding the tag if it happens to be visible from the starting origin.

3.  **`RetryUntilSuccessful` (The Main Search Loop):** The tree enters a loop that will repeat forever until its child succeeds. This makes the search pattern persistent.

4.  **`Parallel` (Simultaneous Navigation & Perception):** Inside the loop, two actions are run at the same time:
    *   **`NavigateThroughPoses`:** This Nav2 action commands the robot to follow the `patrol_route` waypoints sequentially.
    *   **`FindTagAction`:** At the same time, this custom action continuously scans the environment for the destination AprilTag (ID 5). If the tag is found, its pose is saved to the Blackboard under the key `destination_pose`.

5.  **Success Condition:** The `Parallel` node is configured to succeed as soon as **one** of its children succeeds. Since `NavigateThroughPoses` is a continuous action, the only way for the `Parallel` node to succeed is for `FindTagAction` to find the tag.

6.  **`CancelAllNav2`:** Once the `Parallel` node succeeds (meaning the tag was found), this action is called to immediately stop all active Nav2 navigation goals. This halts the patrol instantly.

7.  **`NavigateToPose` (Final Approach):** Finally, the robot navigates to the `destination_pose` that was saved on the Blackboard by the `FindTagAction`.

This architecture creates a robust and flexible mission where the robot actively patrols a predefined area while simultaneously searching for its objective, and then proceeds to the objective once found.

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

**Basic Simulation (Robot Model and Controllers Only):**
  ```bash
  ros2 launch shelfbot gazebo_sim.launch.py
  ```

**Full Autonomous Navigation Simulation (Gazebo + Nav2 + RTAB-Map):**
  ```bash
  ros2 launch shelfbot nav2_dynamic.launch.py
  ```

**Real Robot Operation**

To operate the real robot, you need to launch the hardware drivers and the mapping/localization system (RTAB-Map) in separate terminals.

*Note: The `nav2_real_robot.launch.py` file is currently misnamed. It only launches the robot's hardware drivers and camera processing, not the Nav2 stack.*

1.  **Terminal 1: Launch Hardware Drivers**
    This command starts the connection to the robot's microcontroller and publishes sensor data.
    ```bash
    ros2 launch shelfbot nav2_real_robot.launch.py
    ```

2.  **Terminal 2: Launch RTAB-Map for SLAM**
    This command starts the RTAB-Map node in RGB-only mode, which will use the camera and odometry data to build a map.
    ```bash
    ros2 launch shelfbot rtabmap_rgb_only.launch.py
    ```

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

### `/mission_control_node` (Behavior Tree Engine)
This node executes the high-level mission logic from the `mission.xml` file.

- **Publishers**:
  - `/four_wheel_drive_controller/cmd_vel` (`geometry_msgs/msg/Twist`): Publishes velocity commands when executing `MoveAction` or `SpinAction` nodes from the behavior tree.
- **Subscribers**:
  - `/tf` (`tf2_msgs/msg/TFMessage`): The `FindTagAction` node listens to TF to determine if a specific AprilTag is visible.

### Data Flow and Dependencies
1.  The **Camera** (either simulated in Gazebo or a real hardware driver) publishes images to `/camera/image_raw`.
2.  The `/apriltag_detector_node` subscribes to these images, detects tags, and publishes their 3D poses to `/tf` and `/tag_poses`.
3.  The **Nav2** stack or the **`mission_control_node`** consumes `/tf` data and `/odom` data to make decisions and issue velocity commands.
4.  These commands are published to `/four_wheel_drive_controller/cmd_vel`.
5.  The `/controller_manager` receives these commands and passes them to the `FourWheelDriveHardwareInterface`, which sends the appropriate signals to the motors (either in simulation or via the Micro-ROS agent on the real robot).
6.  The hardware interface reads the wheel encoders, calculates odometry, and publishes it to `/odom` and `/tf`. It also provides joint states to the `/controller_manager`.
7.  The `/controller_manager` publishes the joint states to `/joint_states`.
8.  The `/robot_state_publisher` consumes `/joint_states` to publish the robot's internal transforms to `/tf`.

## Topic Data Structures

This section provides a detailed breakdown of the message structures for the primary topics used in the Shelfbot system, as defined and used within the source code.

### Control & Actuation Topics

#### `/four_wheel_drive_controller/cmd_vel`
- **Type:** `geometry_msgs/msg/Twist`
- **Purpose:** High-level command for robot motion.
- **Structure:**
  ```c++
  // geometry_msgs/msg/Vector3
  struct Vector3 {
    double x, y, z;
  };

  // geometry_msgs/msg/Twist
  struct Twist {
    Vector3  linear;  // linear.x is used for forward/backward velocity
    Vector3  angular; // angular.z is used for rotational velocity
  };
  ```

#### `/shelfbot_firmware/set_speed`
- **Type:** `std_msgs/msg/Float32MultiArray`
- **Purpose:** Low-level command sent to the ESP32 firmware.
- **Structure:**
  ```c++
  // std_msgs/msg/MultiArrayLayout
  struct MultiArrayLayout {
    MultiArrayDimension[] dim;
    uint32 data_offset;
  };

  // std_msgs/msg/Float32MultiArray
  struct Float32MultiArray {
    MultiArrayLayout layout;
    float[]          data; // A vector of 4 floats for motor speeds in rad/s
  };
  ```

### State & Odometry Topics

#### `/odom`
- **Type:** `nav_msgs/msg/Odometry`
- **Purpose:** Provides the robot's estimated position and velocity.
- **Structure:**
  ```c++
  // nav_msgs/msg/Odometry
  struct Odometry {
    std_msgs/msg/Header                  header;
    string                               child_frame_id;
    geometry_msgs/msg/PoseWithCovariance pose;
    geometry_msgs/msg/TwistWithCovariance twist;
  };
  ```

#### `/joint_states`
- **Type:** `sensor_msgs/msg/JointState`
- **Purpose:** Publishes the current angle of all wheel joints.
- **Structure:**
  ```c++
  // sensor_msgs/msg/JointState
  struct JointState {
    std_msgs/msg/Header header;
    string[]            name;     // Names of the joints
    float64[]           position; // Position of the joints in radians
    float64[]           velocity; // Velocity of the joints (optional)
    float64[]           effort;   // Effort of the joints (optional)
  };
  ```

#### `/tf`
- **Type:** `tf2_msgs/msg/TFMessage`
- **Purpose:** Broadcasts all coordinate frame transformations.
- **Structure:**
  ```c++
  // tf2_msgs/msg/TFMessage
  struct TFMessage {
    geometry_msgs/msg/TransformStamped[] transforms;
  };
  ```

### Perception Topics

#### `/camera/image_raw`
- **Type:** `sensor_msgs/msg/Image`
- **Purpose:** Provides raw image data from the camera.
- **Structure:**
  ```c++
  // sensor_msgs/msg/Image
  struct Image {
    std_msgs/msg/Header header;
    uint32              height;
    uint32              width;
    string              encoding; // e.g., "mono8", "bgr8"
    uint8               is_bigendian;
    uint32              step;
    uint8[]             data;
  };
  ```

#### `/camera/camera_info`
- **Type:** `sensor_msgs/msg/CameraInfo`
- **Purpose:** Provides the camera's intrinsic calibration parameters.
- **Structure:**
  ```c++
  // sensor_msgs/msg/CameraInfo
  struct CameraInfo {
    std_msgs/msg/Header header;
    uint32              height;
    uint32              width;
    string              distortion_model;
    float64[]           d; // Distortion coefficients
    float64[9]          k; // Intrinsic camera matrix
    float64[9]          r; // Rectification matrix
    float64[12]         p; // Projection matrix
  };
  ```

#### `/tag_poses`
- **Type:** `geometry_msgs/msg/PoseArray`
- **Purpose:** Publishes the poses of all detected AprilTags.
- **Structure:**
  ```c++
  // geometry_msgs/msg/PoseArray
  struct PoseArray {
    std_msgs/msg/Header header;
    geometry_msgs/msg/Pose[] poses; // Vector of poses for each detected tag
  };
  ```

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
