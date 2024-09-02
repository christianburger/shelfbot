# src/hardware/CMakeLists.txt
add_library(shelfbot_hardware
  hardware_interface.cpp
  camera_driver.cpp
)

ament_target_dependencies(shelfbot_hardware
  rclcpp
  hardware_interface
  sensor_msgs
)

install(TARGETS shelfbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
