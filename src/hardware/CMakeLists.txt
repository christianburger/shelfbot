add_library(hardware_components SHARED
  camera_driver.cpp
  motor_controller.cpp
  hardware_interface.cpp
  encoder_driver.cpp
)

ament_target_dependencies(hardware_components
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS
  hardware_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
