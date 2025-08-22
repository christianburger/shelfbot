# Shelfbot Project

Shelfbot is a ROS2 application of a delivery robot.

The Shelfbot project is a comprehensive robotic system designed to simulate and control a four-wheeled robot. This project includes a detailed robot model, simulation environments, and control interfaces that allow for both simulated and real-time operation. The Shelfbot is equipped with a variety of components, including wheels, motors, and sensors, and is capable of performing complex maneuvers and tasks.

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

## Building the Project

To build the Shelfbot project, follow these steps:

1. **Install ROS 2**: Ensure that ROS 2 is installed and sourced on your system.
2. **Clone the Repository**: Clone the Shelfbot repository into your workspace.
3. **Build the Project**: Use `colcon` to build the project:
  colcon build

4. Source the Setup File: After building, source the setup file to overlay the workspace: 
  source install/setup.bash

## Launching the Simulation

To launch the Shelfbot in a simulation environment, use the provided launch files:

Gazebo Simulation: Launch the robot in Gazebo using: 
  ros2 launch shelfbot gazebo_sim.launch.py

Isaac Sim: Launch the robot in Isaac Sim using: 
  ros2 launch shelfbot isaac_sim_shelfbot_launch.py

RViz Visualization: Visualize the robot in RViz using: 
  ros2 launch shelfbot rviz_launch.py

Note: The Gazebo and Isaac Sim launchers automatically start RViz.

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