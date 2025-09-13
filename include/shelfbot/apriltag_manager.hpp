#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <iomanip>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace shelfbot {

// Placeholder for tag metadata and lifecycle info
struct TagInfo {
    // Will hold: id, is_reference, info, last_seen, ttl, pose, etc.
};

class AprilTagManager {
public:
    explicit AprilTagManager(rclcpp::Node *node);

    // Called when new detections arrive
    void updateTags(const geometry_msgs::msg::PoseArray &pose_array, const std_msgs::msg::Header &header, const std::vector<int> &ids, double tag_size);

private:
    void publishTransforms(const geometry_msgs::msg::PoseArray &pose_array, const std_msgs::msg::Header &header, const std::vector<int> &ids);

    void publishMarkers(const geometry_msgs::msg::PoseArray &pose_array, const std_msgs::msg::Header &header, const std::vector<int> &ids, double tag_size);

private:
    rclcpp::Node *node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unordered_map<int, TagInfo> tags_;
};

} // namespace shelfbot
