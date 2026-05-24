cmake_minimum_required(VERSION 3.8)
project(shelfbot)

# ── Common packages ───────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)

# NOTE: nav2_bringup and rtabmap_ros are runtime-only packages.
# They provide executables, not libraries to link against.
# They are declared in package.xml so rosdep installs them,
# but they must NOT appear here in find_package().

# ── Global include directory ──────────────────────────────────────────────────
include_directories(include)

# ── Sub-directories ───────────────────────────────────────────────────────────
add_subdirectory(src/utils)
add_subdirectory(src/hardware)
add_subdirectory(src/navigation)
add_subdirectory(src/control)
add_subdirectory(src/perception)
add_subdirectory(src/nodes)

# ── Hardware Interface plugin (must be in root CMake to access all sub-libs) ──
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

# ── Installation ─────────────────────────────────────────────────────────────
install(DIRECTORY include/shelfbot DESTINATION include)
install(DIRECTORY urdf    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/mission_starter.py scripts/topic_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    four_wheel_drive_controller_plugin.xml
    four_wheel_drive_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface four_wheel_drive_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface   four_wheel_drive_hardware_interface_plugin.xml)

install(TARGETS four_wheel_drive_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ── Exports ───────────────────────────────────────────────────────────────────
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
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  controller_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
