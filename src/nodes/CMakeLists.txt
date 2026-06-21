# ── SLAM Pose Bridge Node ──────────────────────────────────────────────────────
add_executable(slam_pose_bridge_node
    slam_pose_bridge_node.cpp
)

ament_target_dependencies(slam_pose_bridge_node
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

install(TARGETS
    slam_pose_bridge_node
    DESTINATION lib/${PROJECT_NAME}
)