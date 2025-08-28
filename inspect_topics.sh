#!/bin/bash

# This script iterates through a list of ROS2 message types and
# prints the structure for each one.

# Create an array of unique message types to inspect.
MESSAGE_TYPES=(
    "sensor_msgs/msg/CameraInfo"
    "sensor_msgs/msg/Image"
    "geometry_msgs/msg/PointStamped"
    "apriltag_msgs/msg/AprilTagDetectionArray"
    "diagnostic_msgs/msg/DiagnosticArray"
    "control_msgs/msg/DynamicJointState"
    "geometry_msgs/msg/Twist"
    "std_msgs/msg/Float64MultiArray"
    "lifecycle_msgs/msg/TransitionEvent"
    "geometry_msgs/msg/PoseStamped"
    "sensor_msgs/msg/NavSatFix"
    "sensor_msgs/msg/Imu"
    "geometry_msgs/msg/PoseWithCovarianceStamped"
    "sensor_msgs/msg/JointState"
    "nav_msgs/msg/Odometry"
    "rcl_interfaces/msg/ParameterEvent"
    "std_msgs/msg/String"
    "rcl_interfaces/msg/Log"
    "aruco_msgs/msg/MarkerArray"
    "aruco_opencv_msgs/msg/ArucoDetection"
    "sensor_msgs/msg/PointCloud2"
    "nav_msgs/msg/Path"
    "rtabmap_msgs/msg/Path"
    "rtabmap_msgs/msg/Goal"
    "std_msgs/msg/Bool"
    "nav_msgs/msg/OccupancyGrid"
    "rtabmap_msgs/msg/Info"
    "visualization_msgs/msg/MarkerArray"
    "rtabmap_msgs/msg/LandmarkDetection"
    "rtabmap_msgs/msg/LandmarkDetections"
    "geometry_msgs/msg/PoseArray"
    "rtabmap_msgs/msg/MapData"
    "rtabmap_msgs/msg/MapGraph"
    "octomap_msgs/msg/Octomap"
    "rtabmap_msgs/msg/OdomInfo"
    "rtabmap_msgs/msg/RGBDImage"
    "rtabmap_msgs/msg/SensorData"
    "std_msgs/msg/Int32MultiArray"
    "std_msgs/msg/Float32MultiArray"
    "std_msgs/msg/Int32"
    "tf2_msgs/msg/TFMessage"
    "rtabmap_msgs/msg/UserData"
)

# Loop through the array and print the interface for each message type.
for msg_type in "${MESSAGE_TYPES[@]}"; do
  echo "--- ${msg_type} ---"
  ros2 interface show "${msg_type}"
  echo
done
