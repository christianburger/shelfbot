find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(four_wheel_drive_odometry SHARED
  four_wheel_drive_odometry.cpp
)

target_include_directories(four_wheel_drive_odometry PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/shelfbot>
)

ament_target_dependencies(four_wheel_drive_odometry
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  four_wheel_drive_odometry
  DESTINATION lib
)