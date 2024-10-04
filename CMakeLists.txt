cmake_minimum_required(VERSION 3.5)
project(shelfbot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
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

# Add the ShelfbotUtils library
add_library(shelfbot_utils SHARED
  src/shelfbot_utils.cpp
)

target_include_directories(shelfbot_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(shelfbot_utils
  rclcpp
)

# Add the FourWheelDriveController library
add_library(four_wheel_drive_controller SHARED
  src/control/four_wheel_drive_controller.cpp
)

target_include_directories(four_wheel_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(four_wheel_drive_controller
  shelfbot_utils
)

ament_target_dependencies(four_wheel_drive_controller
  hardware_interface
  pluginlib
  rclcpp
  controller_interface
  gazebo_ros2_control
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(controller_interface four_wheel_drive_controller_plugin.xml)

# Install targets
install(TARGETS
  shelfbot_utils
  four_wheel_drive_controller
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