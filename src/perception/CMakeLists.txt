# Find all necessary packages for the perception node
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the executable target
add_executable(apriltag_detector_node apriltag_detector.cpp)

# Explicitly add the include directories from all dependencies to the target.
target_include_directories(apriltag_detector_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${apriltag_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Use the standard ament macro to handle library linking for ROS dependencies.
ament_target_dependencies(apriltag_detector_node
    rclcpp
    image_transport
    cv_bridge
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    sensor_msgs
    geometry_msgs
)

# Explicitly link the core apriltag library
target_link_libraries(apriltag_detector_node apriltag::apriltag)

# Install the executable
install(TARGETS
  apriltag_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
