cmake_minimum_required(VERSION 3.5)
project(shelfbot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Add executables
add_executable(camera_driver src/hardware/camera_driver.cpp)
ament_target_dependencies(camera_driver rclcpp sensor_msgs)

add_executable(hardware_interface src/hardware/hardware_interface.cpp)
ament_target_dependencies(hardware_interface rclcpp tf2_ros)

# Add the Gazebo plugin
add_library(gazebo_ros_four_wheel_drive SHARED
  src/simulation/gazebo_ros_four_wheel_drive.cpp
)
target_link_libraries(gazebo_ros_four_wheel_drive ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_ros_four_wheel_drive
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS
  camera_driver
  hardware_interface
  gazebo_ros_four_wheel_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
