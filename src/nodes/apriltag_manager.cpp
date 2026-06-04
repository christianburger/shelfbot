#include "shelfbot/apriltag_manager.hpp"
#include "shelfbot/shelfbot_utils.hpp"

#include <sstream>
#include <iomanip>

namespace shelfbot {

AprilTagManager::AprilTagManager(rclcpp::Node* node) : node_(node) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // RELIABLE, VOLATILE, depth=10 — matches what ORB-SLAM3 / Nav2 subscribers
    // use.  The original SensorDataQoS (BEST_EFFORT) would silently fail to
    // deliver to any RELIABLE downstream consumer.
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                            .reliable()
                            .durability_volatile();

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/apriltag_markers", reliable_qos);

    pose_array_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        "/tag_poses", reliable_qos);

    shelfbot::log_info(node_->get_name(), "Constructor",
        "Publishers created: /tag_poses, /apriltag_markers  (RELIABLE, VOLATILE)");
}

void AprilTagManager::updateTags(
    const geometry_msgs::msg::PoseArray& pose_array,
    const std_msgs::msg::Header&         header,
    const std::vector<int>&              ids,
    double                               tag_size)
{
    pose_array_msg_        = pose_array;
    pose_array_msg_.header = header;   // preserves SNTP epoch stamp from image
    pose_array_pub_->publish(pose_array_msg_);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "Published /tag_poses with " << pose_array.poses.size() << " tags"
        << "  stamp=" << header.stamp.sec << "." << header.stamp.nanosec;
    shelfbot::log_info(node_->get_name(), "UpdateTags", oss.str());

    publishTransforms(pose_array, header, ids);
    publishMarkers(pose_array, header, ids, tag_size);
}

void AprilTagManager::publishTransforms(
    const geometry_msgs::msg::PoseArray& pose_array,
    const std_msgs::msg::Header&         header,
    const std::vector<int>&              ids)
{
    for (size_t i = 0; i < pose_array.poses.size(); ++i) {
        geometry_msgs::msg::TransformStamped ts;
        ts.header         = header;
        ts.child_frame_id = "tag_" + std::to_string(ids[i]);
        ts.transform.translation.x = pose_array.poses[i].position.x;
        ts.transform.translation.y = pose_array.poses[i].position.y;
        ts.transform.translation.z = pose_array.poses[i].position.z;
        ts.transform.rotation      = pose_array.poses[i].orientation;
        tf_broadcaster_->sendTransform(ts);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "TF tag_" << ids[i]
            << " pos=(" << ts.transform.translation.x
            << ", "     << ts.transform.translation.y
            << ", "     << ts.transform.translation.z << ")";
        shelfbot::log_info(node_->get_name(), "PublishTransforms", oss.str());
    }
}

void AprilTagManager::publishMarkers(
    const geometry_msgs::msg::PoseArray& pose_array,
    const std_msgs::msg::Header&         header,
    const std::vector<int>&              ids,
    double                               tag_size)
{
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < pose_array.poses.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header   = header;
        marker.ns       = "apriltag";
        marker.id       = ids[i];
        marker.type     = visualization_msgs::msg::Marker::CUBE;
        marker.action   = visualization_msgs::msg::Marker::ADD;
        marker.pose     = pose_array.poses[i];
        marker.scale.x  = tag_size;
        marker.scale.y  = tag_size;
        marker.scale.z  = 0.1 * tag_size;
        marker.color.a  = 0.5f;
        marker.color.r  = 0.0f;
        marker.color.g  = 1.0f;
        marker.color.b  = 0.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}

} // namespace shelfbot
