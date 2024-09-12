cmake_minimum_required(VERSION 3.5)
project(shelfbot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz2 REQUIRED)

# Include directories
include_directories(
  include
)

# Add executables
add_executable(camera_driver src/hardware/camera_driver.cpp)
ament_target_dependencies(camera_driver rclcpp sensor_msgs)

add_executable(hardware_interface src/hardware/hardware_interface.cpp)
ament_target_dependencies(hardware_interface rclcpp tf2_ros)

# Install executables
install(TARGETS
  camera_driver
  hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (for RViz)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ament package for ROS 2 dependencies
ament_package()