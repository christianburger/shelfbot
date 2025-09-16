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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <sophus/se3.hpp>  // For Sophus::SE3f
#include <Eigen/Core>      // For Eigen::Matrix4f (already indirect via Sophus, but explicit is safer)
#include "Tracking.h"      // For ORB_SLAM3::Tracking::OK enum

// ORB-SLAM3 core
#include "System.h"

class ShelfbotORB3Node : public rclcpp::Node {
public:
  ShelfbotORB3Node(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("shelfbot_slam_orb3_node", opts), slam_initialized_(false), last_pose_valid_(false)
  {
    // Declare parameters with sensible defaults
    declare_parameter<std::string>("voc_file", "/path/to/ORBvoc.txt");
    declare_parameter<std::string>("settings_file", "/path/to/config.yaml");
    declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    declare_parameter<std::string>("camera_frame", "camera_link");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_link_frame", "base_link");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("publish_odom", true);
    declare_parameter<double>("tracking_lost_timeout", 5.0);

    get_parameter("voc_file", voc_file_);
    get_parameter("settings_file", settings_file_);
    get_parameter("camera_topic", camera_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("map_frame", map_frame_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("base_link_frame", base_link_frame_);
    get_parameter("publish_tf", publish_tf_);
    get_parameter("publish_odom", publish_odom_);
    get_parameter("tracking_lost_timeout", tracking_lost_timeout_);

    // Validate required parameters
    if (voc_file_.empty() || settings_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "voc_file and settings_file parameters are required!");
      rclcpp::shutdown();
      return;
    }

    // Initialize ORB-SLAM3 system
    try {
      slam_system_ = std::make_shared<ORB_SLAM3::System>(
        voc_file_, settings_file_,
        ORB_SLAM3::System::MONOCULAR,
        false  // no Pangolin viewer window for production
      );
      slam_initialized_ = true;
      RCLCPP_INFO(get_logger(), "ORB-SLAM3 system initialized successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // Set up synchronized subscribers for image and camera info
    image_sub_.subscribe(this, camera_topic_, rmw_qos_profile_sensor_data);
    camera_info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);
    
    // Synchronize image and camera_info messages
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_sub_, camera_info_sub_);
    sync_->registerCallback(std::bind(&ShelfbotORB3Node::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Publishers for nav2 compatibility
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }
    
    if (publish_odom_) {
      odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("slam_odom", rclcpp::QoS(10));
    }

    // Publisher for initial pose (for nav2 localization)
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(1));

    // Status tracking
    last_tracking_time_ = now();
    
    RCLCPP_INFO(get_logger(),
      "ShelfBot SLAM initialized. Image: [%s], CameraInfo: [%s]",
      camera_topic_.c_str(), camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), 
      "Publishing TF: %s, Publishing Odom: %s", 
      publish_tf_ ? "Yes" : "No", publish_odom_ ? "Yes" : "No");
  }

  ~ShelfbotORB3Node() {
    if (slam_system_) {
      RCLCPP_INFO(get_logger(), "Shutting down ORB-SLAM3...");
      slam_system_->Shutdown();
    }
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
    
    if (!slam_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "SLAM not initialized yet");
      return;
    }

    // Convert image to grayscale (mono8) using cv_bridge for automatic handling of encodings
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat& gray_image = cv_ptr->image;
    if (gray_image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty grayscale image");
      return;
    }

    // Use message timestamp for SLAM (seconds since epoch)
    const double timestamp = rclcpp::Time(image_msg->header.stamp).seconds();
    
    // Run SLAM tracking (returns camera-to-world pose as Sophus::SE3f)
    const Sophus::SE3f pose_se3 = slam_system_->TrackMonocular(gray_image, timestamp);
    
    // Check tracking status using ORB-SLAM3's internal state
    const int tracking_state = slam_system_->GetTrackingState();
    const bool tracking_successful = (tracking_state == static_cast<int>(ORB_SLAM3::Tracking::OK));
    
    const auto current_time = this->now();
    
    if (tracking_successful) {
      last_tracking_time_ = current_time;
      last_pose_valid_ = true;
      
      // Convert Sophus::SE3f to cv::Mat (4x4, CV_32F) for legacy processing
      cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
      const Eigen::Matrix4f eigen_Tcw = pose_se3.matrix();
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          Tcw.at<float>(r, c) = eigen_Tcw(r, c);
        }
      }
      
      // Process successful tracking
      processSuccessfulTracking(Tcw, image_msg->header.stamp, camera_info_msg);
      
    } else {
      // Handle tracking loss with timeout-based warning
      const double time_since_last_track = (current_time - last_tracking_time_).seconds();
      if (time_since_last_track > tracking_lost_timeout_) {
        if (last_pose_valid_) {
          RCLCPP_WARN(get_logger(), "SLAM tracking lost for %.1f seconds (state: %d)", 
                      time_since_last_track, tracking_state);
          last_pose_valid_ = false;
        }
      }
    }
  }

  void processSuccessfulTracking(const cv::Mat& Tcw, const builtin_interfaces::msg::Time& stamp,
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    // Convert camera pose to world pose (Twc = Tcw^-1)
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc * tcw;

    // Convert to tf2 types
    tf2::Matrix3x3 tf_rotation(
      Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
      Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
      Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2)
    );
    
    tf2::Quaternion tf_quaternion;
    tf_rotation.getRotation(tf_quaternion);

    tf2::Vector3 tf_translation(
      twc.at<float>(0), twc.at<float>(1), twc.at<float>(2)
    );

    // Publish TF transform (map -> camera_link)
    if (publish_tf_) {
      publishTransform(tf_translation, tf_quaternion, stamp);
    }

    // Publish odometry for nav2
    if (publish_odom_) {
      publishOdometry(tf_translation, tf_quaternion, stamp);
    }

    // Log pose occasionally
    static int pose_count = 0;
    if (++pose_count % 30 == 1) {  // Every ~1 second at 30fps
      RCLCPP_INFO(get_logger(), 
        "SLAM pose: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
        tf_translation.x(), tf_translation.y(), tf_translation.z(),
        tf_quaternion.x(), tf_quaternion.y(), tf_quaternion.z(), tf_quaternion.w());
    }
  }

  void publishTransform(const tf2::Vector3& translation, const tf2::Quaternion& rotation,
                       const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = map_frame_;
    transform_msg.child_frame_id = camera_frame_;
    
    transform_msg.transform.translation.x = translation.x();
    transform_msg.transform.translation.y = translation.y();
    transform_msg.transform.translation.z = translation.z();
    transform_msg.transform.rotation.x = rotation.x();
    transform_msg.transform.rotation.y = rotation.y();
    transform_msg.transform.rotation.z = rotation.z();
    transform_msg.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_msg);
  }

  void publishOdometry(const tf2::Vector3& translation, const tf2::Quaternion& rotation,
                      const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = camera_frame_;

    // Position
    odom_msg.pose.pose.position.x = translation.x();
    odom_msg.pose.pose.position.y = translation.y();
    odom_msg.pose.pose.position.z = translation.z();
    
    // Orientation
    odom_msg.pose.pose.orientation.x = rotation.x();
    odom_msg.pose.pose.orientation.y = rotation.y();
    odom_msg.pose.pose.orientation.z = rotation.z();
    odom_msg.pose.pose.orientation.w = rotation.w();

    // Covariance (conservative estimates for SLAM uncertainty)
    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[14] = 0.1;  // z
    odom_msg.pose.covariance[21] = 0.05; // roll
    odom_msg.pose.covariance[28] = 0.05; // pitch
    odom_msg.pose.covariance[35] = 0.05; // yaw

    // Velocity (ORB-SLAM3 doesn't directly provide velocity, set to zero)
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Conservative velocity covariance
    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    odom_msg.twist.covariance[0] = 0.1;  // vx
    odom_msg.twist.covariance[7] = 0.1;  // vy
    odom_msg.twist.covariance[35] = 0.1; // angular z

    odom_publisher_->publish(odom_msg);
  }

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
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

  // State tracking
  bool last_pose_valid_;
  rclcpp::Time last_tracking_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<ShelfbotORB3Node>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
