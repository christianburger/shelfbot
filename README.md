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

## Hardware Interface

The Shelfbot includes a hardware interface for controlling and monitoring the robot's state:

Command Interfaces: Allow sending commands to the robot's joints and actuators.
State Interfaces: Provide feedback on the robot's current state, including joint positions and velocities.

## Using ROS 2 Command Line Tools

To interact with the hardware interface, you can use ROS 2 command line tools:

Send Commands: Use ros2 topic pub to send commands to the robot's joints:

  ros2 topic pub /shelfbot/commands std_msgs/msg/Float64MultiArray "data: [1.0, 0.5, -0.5, -1.0]"

Receive State Information: Use ros2 topic echo to view the robot's state information:

  ros2 topic echo /shelfbot/joint_states

## Conclusion

The Shelfbot project provides a versatile platform for simulating and controlling a four-wheeled robot.
With its integration into ROS 2, it offers a robust framework for developing and testing robotic applications in both simulated and real-world environments.