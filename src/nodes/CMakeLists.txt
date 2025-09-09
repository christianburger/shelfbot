find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Camera Publisher Node
add_executable(camera_publisher camera_publisher.cpp)
ament_target_dependencies(camera_publisher
        rclcpp
        sensor_msgs
        camera_info_manager
)

# AprilTag Detector Node
add_executable(apriltag_detector_node apriltag_detector.cpp)
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

# Link apriltag library and set include directories
target_link_libraries(apriltag_detector_node apriltag::apriltag)
target_include_directories(apriltag_detector_node PRIVATE
        /opt/ros/humble/include/apriltag
)

install(TARGETS
        camera_publisher
        apriltag_detector_node
        DESTINATION lib/${PROJECT_NAME}
)