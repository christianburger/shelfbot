# src/monitoring/CMakeLists.txt

add_executable(monitoring_node monitoring_node.cpp)

ament_target_dependencies(monitoring_node
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS monitoring_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
