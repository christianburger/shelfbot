find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(microros_communication SHARED
  microros_communication.cpp
)

target_include_directories(microros_communication PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/shelfbot>
)

target_link_libraries(microros_communication
  shelfbot_utils
)

ament_target_dependencies(microros_communication
  rclcpp
  std_msgs
)

install(TARGETS microros_communication
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
