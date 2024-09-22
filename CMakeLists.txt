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
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
# Add these lines
add_library(four_wheel_drive_controller SHARED
  src/control/four_wheel_drive_controller.cpp
)

target_include_directories(four_wheel_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(four_wheel_drive_controller
  hardware_interface
  pluginlib
  rclcpp
  controller_interface
)

pluginlib_export_plugin_description_file(hardware_interface four_wheel_drive_controller_plugin.xml)

install(TARGETS four_wheel_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
  four_wheel_drive_controller
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
  include
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
