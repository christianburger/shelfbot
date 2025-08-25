add_library(four_wheel_drive_odometry SHARED
  four_wheel_drive_odometry.cpp
)

target_include_directories(four_wheel_drive_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(four_wheel_drive_odometry
  shelfbot_utils
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
)

ament_target_dependencies(four_wheel_drive_odometry
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- Mission Control Node ---
add_executable(mission_control_node path_planner.cpp)

ament_target_dependencies(mission_control_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

install(TARGETS
  mission_control_node
  DESTINATION lib/${PROJECT_NAME}
)
