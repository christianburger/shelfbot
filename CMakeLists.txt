cmake_minimum_required(VERSION 3.8)
project(shelfbot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
)

# Add subdirectories for each module
add_subdirectory(src/hardware)
add_subdirectory(src/simulation)
add_subdirectory(src/monitoring)

# Install URDF and launch files
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

