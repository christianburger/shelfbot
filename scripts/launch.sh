#!/bin/bash
source ./install/setup.bash; ros2 launch shelfbot real_robot_hardened_lidar_microros.launch.py | tee src/shelfbot/log/launch.log
