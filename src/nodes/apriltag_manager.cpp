#include "shelfbot/apriltag_manager.hpp"
#include "shelfbot/shelfbot_utils.hpp"

#include <sstream>
#include <iomanip>

namespace shelfbot {

AprilTagManager::AprilTagManager(rclcpp::Node *node) : node_(node) {
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("apriltag_markers", 10);
  pose_array_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("tag_poses", rclcpp::SensorDataQoS());
  shelfbot::log_info(node_->get_name(), "Constructor", "Centralized publishers created: tag_poses, apriltag_markers, TF transforms");
}

void AprilTagManager::updateTags(const geometry_msgs::msg::PoseArray &pose_array, const std_msgs::msg::Header &header, const std::vector<int> &ids, double tag_size) {
  // Publish PoseArray first, then transforms and markers
  pose_array_msg_ = pose_array;
  pose_array_msg_.header = header;
  pose_array_pub_->publish(pose_array_msg_);

  std::ostringstream oss_pose;
  oss_pose << std::fixed << std::setprecision(3) << "Published PoseArray with " << pose_array.poses.size() << " tags";
  shelfbot::log_info(node_->get_name(), "UpdateTags", oss_pose.str());

  publishTransforms(pose_array, header, ids);
  publishMarkers(pose_array, header, ids, tag_size);
}

void AprilTagManager::publishTransforms(const geometry_msgs::msg::PoseArray &pose_array, const std_msgs::msg::Header &header, const std::vector<int> &ids) {
    for (size_t i = 0; i < pose_array.poses.size(); ++i) {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header = header;
        transformStamped.child_frame_id = "tag_" + std::to_string(ids[i]);
        transformStamped.transform.translation.x = pose_array.poses[i].position.x;
        transformStamped.transform.translation.y = pose_array.poses[i].position.y;
        transformStamped.transform.translation.z = pose_array.poses[i].position.z;
        transformStamped.transform.rotation = pose_array.poses[i].orientation;

        tf_broadcaster_->sendTransform(transformStamped);

        std::ostringstream oss_tf;
        oss_tf << std::fixed << std::setprecision(3)
               << "Published transform for tag_" << ids[i]
               << ": pos=("
               << transformStamped.transform.translation.x << ", "
               << transformStamped.transform.translation.y << ", "
               << transformStamped.transform.translation.z << ")";
        shelfbot::log_info(node_->get_name(), "PublishTransforms", oss_tf.str());
    }
}

void AprilTagManager::publishMarkers(const geometry_msgs::msg::PoseArray &pose_array, const std_msgs::msg::Header &header, const std::vector<int> &ids, double tag_size) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < pose_array.poses.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "apriltag";
        marker.id = ids[i];
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose_array.poses[i];
        marker.scale.x = tag_size;
        marker.scale.y = tag_size;
        marker.scale.z = 0.1 * tag_size;
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.0); // Persistent

        marker_array.markers.push_back(marker);

        std::ostringstream oss_marker;
        oss_marker << std::fixed << std::setprecision(3)
                   << "Published marker for tag_" << ids[i]
                   << ": scale=("
                   << marker.scale.x << ", "
                   << marker.scale.y << ", "
                   << marker.scale.z << ")";
        shelfbot::log_info(node_->get_name(), "PublishMarkers", oss_marker.str());
    }
    marker_pub_->publish(marker_array);
}

} // namespace shelfbot