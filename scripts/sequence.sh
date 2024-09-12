#! /bin/bash

echo 'rm build log install -rf; colcon build; source install/setup.bash'
rm build log install -rf; 
colcon build; 
source install/setup.bash

echo 'xacro ~/ros2_ws/src/shelfbot/urdf/shelfbot.urdf.xacro -o /tmp/shelfbot.urdf'
xacro ~/ros2_ws/src/shelfbot/urdf/shelfbot.urdf.xacro -o /tmp/shelfbot.urdf

echo 'check_urdf /tmp/shelfbot.urdf'
check_urdf /tmp/shelfbot.urdf
