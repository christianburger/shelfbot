#! /bin/bash
# Run the robot_state_publisher with the XACRO file
echo Run the robot_state_publisher with the XACRO file
echo 'ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/shelfbot/urdf/shelfbot.urdf.xacro)"'

# Run the static_transform_publisher
echo Run the static_transform_publisher
echo 'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint'

# Launch RViz2 with the specified configuration file
echo Launch RViz2 with the specified configuration file
echo 'ros2 run rviz2 rviz2 -d /home/chris/ros2_ws/src/shelfbot/rviz2/shelfbot_config.rviz'

