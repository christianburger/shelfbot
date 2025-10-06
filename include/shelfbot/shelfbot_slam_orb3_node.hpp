#ifndef SHELFBOT_SLAM_ORB3_NODE_HPP
#define SHELFBOT_SLAM_ORB3_NODE_HPP

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include "Tracking.h"

// ORB-SLAM3 core
#include "System.h"

// Shelfbot Utils for logging
#include "shelfbot/shelfbot_utils.hpp"

class ShelfbotORB3Node : public rclcpp::Node {
public:
  ShelfbotORB3Node(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  ~ShelfbotORB3Node();

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  // Utility: convert OpenCV optical coordinates (ORB-SLAM3) to ROS REP-105
  static tf2::Transform convertFromCvToRos(const tf2::Transform& cv_tf);

  void publishSlamOdom(const tf2::Transform& map_to_base_link, const builtin_interfaces::msg::Time& stamp);

  void imageCallbackRaw(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void cameraInfoCallbackRaw(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);
  void processSuccessfulTracking(const cv::Mat& Tcw, const builtin_interfaces::msg::Time& stamp, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);
  void publishMapToOdomTransform(const tf2::Transform& camera_pose_in_map, const builtin_interfaces::msg::Time& stamp);
  void publishInitialPose(const tf2::Transform& map_to_odom, const builtin_interfaces::msg::Time& stamp);

  // Parameters
  std::string voc_file_, settings_file_;
  std::string camera_topic_, camera_info_topic_;
  std::string camera_frame_, map_frame_, odom_frame_, base_link_frame_;
  bool publish_tf_, publish_odom_;
  double tracking_lost_timeout_;

  // ORB-SLAM3
  std::shared_ptr<ORB_SLAM3::System> slam_system_;
  bool slam_initialized_;

  // ROS2 communication
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // TROUBLESHOOT: Raw subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_raw_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_raw_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

  // State tracking
  bool last_pose_valid_;
  rclcpp::Time last_tracking_time_;

  // TROUBLESHOOT: Counters
  int image_count_, camera_info_count_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Additional for logging
  double last_timestamp_ = 0.0;
  cv::Mat last_gray_image_;
};

#endif // SHELFBOT_SLAM_ORB3_NODE_HPP
