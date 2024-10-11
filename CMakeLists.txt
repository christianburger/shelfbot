cmake_minimum_required(VERSION 3.5)

project(shelfbot)

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
find_package(msgpack REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

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

add_library(four_wheel_drive_controller SHARED
  src/control/four_wheel_drive_controller.cpp
)

target_include_directories(four_wheel_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(four_wheel_drive_controller
  shelfbot_utils
  msgpackc
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
  msgpack
)

add_library(four_wheel_drive_hardware_interface SHARED
  src/control/four_wheel_drive_hardware_interface.cpp
)

target_include_directories(four_wheel_drive_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(four_wheel_drive_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(controller_interface four_wheel_drive_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface four_wheel_drive_hardware_interface_plugin.xml)

install(TARGETS
  shelfbot_utils
  four_wheel_drive_controller
  four_wheel_drive_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  include
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
