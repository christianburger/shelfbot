cmake_minimum_required(VERSION 3.8)
project(shelfbot)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgpack REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

# Build utilities
add_library(shelfbot_utils SHARED
  src/shelfbot_utils.cpp
)
target_link_libraries(shelfbot_utils
  ${rclcpp_LIBRARIES}
)
ament_target_dependencies(shelfbot_utils
  rclcpp
)

# Communication libraries
add_library(serial_communication SHARED
  src/hardware/serial_communication.cpp
)
target_link_libraries(serial_communication
  shelfbot_utils
  ${msgpack_LIBRARIES}
)
target_include_directories(serial_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(mock_communication SHARED
  src/hardware/mock_communication.cpp
)
target_link_libraries(mock_communication
  shelfbot_utils
  ${msgpack_LIBRARIES}
)
target_include_directories(mock_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(rest_communication SHARED
  src/hardware/rest_communication.cpp
)
target_link_libraries(rest_communication
  shelfbot_utils
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
  ${msgpack_LIBRARIES}
)
target_include_directories(rest_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Navigation components
add_subdirectory(src/navigation)

# Controller
add_library(four_wheel_drive_controller SHARED
  src/control/four_wheel_drive_controller.cpp
)
target_link_libraries(four_wheel_drive_controller
  shelfbot_utils
  four_wheel_drive_odometry
  ${rclcpp_LIBRARIES}
  ${controller_interface_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${msgpack_LIBRARIES}
)
ament_target_dependencies(four_wheel_drive_controller
  rclcpp
  controller_interface
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msgpack
  sensor_msgs
  rclcpp_lifecycle
)

# Hardware Interface
add_library(four_wheel_drive_hardware_interface SHARED
  src/control/four_wheel_drive_hardware_interface.cpp
)
target_link_libraries(four_wheel_drive_hardware_interface
  shelfbot_utils
  serial_communication
  mock_communication
  rest_communication
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${msgpack_LIBRARIES}
)
ament_target_dependencies(four_wheel_drive_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  msgpack
)

# Installation
install(TARGETS
  shelfbot_utils
  serial_communication
  mock_communication
  rest_communication
  four_wheel_drive_controller
  four_wheel_drive_hardware_interface
  four_wheel_drive_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  four_wheel_drive_controller_plugin.xml
  four_wheel_drive_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(controller_interface four_wheel_drive_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface four_wheel_drive_hardware_interface_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(
  shelfbot_utils 
  serial_communication 
  mock_communication 
  rest_communication
  four_wheel_drive_controller 
  four_wheel_drive_hardware_interface
  four_wheel_drive_odometry
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  controller_interface
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msgpack
  sensor_msgs
  rclcpp_lifecycle
  CURL
  nlohmann_json
)

ament_package()