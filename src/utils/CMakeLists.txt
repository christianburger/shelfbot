find_package(rclcpp REQUIRED)

add_library(shelfbot_utils SHARED
  shelfbot_utils.cpp
)

target_include_directories(shelfbot_utils PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/shelfbot>
)

ament_target_dependencies(shelfbot_utils
  rclcpp
)

install(TARGETS shelfbot_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
