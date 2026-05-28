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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Eigen3 REQUIRED)

# ── ORB-SLAM3 paths ───────────────────────────────────────────────────────────
if(DEFINED ENV{ORB_SLAM3_ROOT})
    set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT}")
else()
    set(ORB_SLAM3_ROOT_DIR "/opt/ORB_SLAM3")
endif()
message(STATUS "ORB_SLAM3_ROOT_DIR = ${ORB_SLAM3_ROOT_DIR}")

find_path(ORB_SLAM3_INCLUDE_DIRS
        NAMES System.h
        PATHS ${ORB_SLAM3_ROOT_DIR}/include
        NO_DEFAULT_PATH
)

find_library(ORB_SLAM3_LIBRARIES
        NAMES ORB_SLAM3
        PATHS ${ORB_SLAM3_ROOT_DIR}/lib
        NO_DEFAULT_PATH
)

find_library(DBoW2_LIBRARIES
        NAMES DBoW2
        PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
        NO_DEFAULT_PATH
)

find_library(g2o_LIBRARIES
        NAMES g2o
        PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
        NO_DEFAULT_PATH
)

# ── Pangolin paths ────────────────────────────────────────────────────────────
find_library(Pangolin_LIBRARIES
        NAMES pangolin
        PATHS /opt/pangolin/lib
        NO_DEFAULT_PATH
)

find_path(Pangolin_INCLUDE_DIRS
        NAMES pangolin/pangolin.h
        PATHS /opt/pangolin/include
        NO_DEFAULT_PATH
)

message(STATUS "ORB_SLAM3_INCLUDE_DIRS = ${ORB_SLAM3_INCLUDE_DIRS}")
message(STATUS "ORB_SLAM3_LIBRARIES    = ${ORB_SLAM3_LIBRARIES}")
message(STATUS "Pangolin_LIBRARIES     = ${Pangolin_LIBRARIES}")
message(STATUS "Pangolin_INCLUDE_DIRS  = ${Pangolin_INCLUDE_DIRS}")
message(STATUS "DBoW2_LIBRARIES        = ${DBoW2_LIBRARIES}")
message(STATUS "g2o_LIBRARIES          = ${g2o_LIBRARIES}")

# ── Camera Publisher Node ─────────────────────────────────────────────────────
add_executable(camera_publisher camera_publisher.cpp)
ament_target_dependencies(camera_publisher
        rclcpp
        sensor_msgs
        camera_info_manager
        cv_bridge
        message_filters
)

# ── AprilTag Detector Node ────────────────────────────────────────────────────
add_executable(apriltag_detector_node
        apriltag_detector.cpp
        apriltag_manager.cpp
)
ament_target_dependencies(apriltag_detector_node
        rclcpp
        image_transport
        cv_bridge
        tf2
        tf2_ros
        tf2_geometry_msgs
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
)
target_link_libraries(apriltag_detector_node
        apriltag::apriltag
        shelfbot_utils
)
target_include_directories(apriltag_detector_node PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        /opt/ros/humble/include/apriltag
)

# ── ORB-SLAM3 SLAM Node ───────────────────────────────────────────────────────
add_executable(shelfbot_slam_orb3_node
        shelfbot_slam_orb3_node.cpp
)

target_include_directories(shelfbot_slam_orb3_node PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ORB_SLAM3_ROOT_DIR}
        ${ORB_SLAM3_INCLUDE_DIRS}
        ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/include
        ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/include
        ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
)

target_link_libraries(shelfbot_slam_orb3_node
        ${ORB_SLAM3_LIBRARIES}
        ${DBoW2_LIBRARIES}
        ${g2o_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        shelfbot_utils
)

ament_target_dependencies(shelfbot_slam_orb3_node
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        cv_bridge
        message_filters
)

# ── Lidar Relay Node ──────────────────────────────────────────────────────────
# Converts /shelfbot_firmware/lidar_scan (Float32MultiArray, 30 floats)
# into /scan (sensor_msgs/LaserScan) for Nav2, RViz2, and rtabmap.
add_executable(lidar_relay_node
        lidar_relay_node.cpp
)

ament_target_dependencies(lidar_relay_node
        rclcpp
        std_msgs
        sensor_msgs
)

target_link_libraries(lidar_relay_node
        shelfbot_utils
)

# ── Install all executables ───────────────────────────────────────────────────
install(TARGETS
        camera_publisher
        apriltag_detector_node
        shelfbot_slam_orb3_node
        lidar_relay_node
        DESTINATION lib/${PROJECT_NAME}
)