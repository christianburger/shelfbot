add_library(four_wheel_drive_controller SHARED
  four_wheel_drive_controller.cpp
)

target_include_directories(four_wheel_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(four_wheel_drive_controller
  rclcpp
  controller_interface
  realtime_tools
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS four_wheel_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  four_wheel_drive_controller.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

