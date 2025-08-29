cmake_minimum_required(VERSION 3.8)
project(shelfbot)

# Find common packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(rtabmap_ros REQUIRED)

# Set the include directory for the whole project
include_directories(include)

# Build components in subdirectories
add_subdirectory(src/utils)
add_subdirectory(src/hardware)
add_subdirectory(src/navigation)
add_subdirectory(src/control)
add_subdirectory(src/perception)
add_subdirectory(src/nodes)

# Controller (Hardware Interface)
add_library(four_wheel_drive_hardware_interface SHARED
  src/control/four_wheel_drive_hardware_interface.cpp
)
target_link_libraries(four_wheel_drive_hardware_interface
  shelfbot_utils
  microros_communication
  four_wheel_drive_odometry
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES}
  ${pluginlib_LIBRARIES}
)
ament_target_dependencies(four_wheel_drive_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
)

# Installation
install(
    DIRECTORY include/shelfbot
    DESTINATION include
)

install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS scripts/mission_starter.py scripts/topic_logger.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES four_wheel_drive_controller_plugin.xml four_wheel_drive_hardware_interface_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface four_wheel_drive_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface four_wheel_drive_hardware_interface_plugin.xml)

install(TARGETS
    four_wheel_drive_hardware_interface
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


ament_export_include_directories(include)
ament_export_libraries(
  shelfbot_utils
  microros_communication
  four_wheel_drive_controller
  four_wheel_drive_hardware_interface
  four_wheel_drive_odometry
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
)

ament_package()