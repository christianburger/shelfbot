add_library(four_wheel_drive_odometry SHARED
  four_wheel_drive_odometry.cpp
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
