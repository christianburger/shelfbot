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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
  ShelfbotORB3Node(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions()) : Node("shelfbot_slam_orb3_node", opts), slam_initialized_(false), last_pose_valid_(false), image_count_(0), camera_info_count_(0) {  // TROUBLESHOOT: Counters for receipt rates
    // Add this to your constructor
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Declare parameters with sensible defaults
    declare_parameter<std::string>("voc_file", "/home/chris/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    declare_parameter<std::string>("settings_file", "/home/chris/shelfbot_workspace/src/shelfbot/config/orb_slam3_monocular.yaml");
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
      RCLCPP_ERROR(get_logger(), "shelfbot_slam_orb3: voc_file and settings_file parameters are required!");
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
      RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: ORB-SLAM3 system initialized successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "shelfbot_slam_orb3: Failed to initialize ORB-SLAM3: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // TROUBLESHOOT: Individual rclcpp subscriptions to check raw message receipt (bypass sync)
    image_sub_raw_ = this->create_subscription<sensor_msgs::msg::Image>( camera_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&ShelfbotORB3Node::imageCallbackRaw, this, std::placeholders::_1));
    camera_info_sub_raw_ = this->create_subscription<sensor_msgs::msg::CameraInfo>( camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&ShelfbotORB3Node::cameraInfoCallbackRaw, this, std::placeholders::_1));

    // Set up synchronized subscribers for image and camera info
    // In shelfbot_slam_orb3_node.cpp, replace the subscription lines
    image_sub_.subscribe(this, camera_topic_, rmw_qos_profile_t{
      .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 10,
      .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE
    });

    camera_info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_t{
      .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 10,
      .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE
    });
    
    // Synchronize image and camera_info messages
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), image_sub_, camera_info_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));  // TROUBLESHOOT: Larger queue, explicit slop
    sync_->registerCallback(std::bind(&ShelfbotORB3Node::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // TROUBLESHOOT: Log after sync registration
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Sync registered with queue=20, slop=0.1s");

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
    
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: ShelfBot SLAM initialized. Image: [%s], CameraInfo: [%s]", camera_topic_.c_str(), camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Publishing TF: %s, Publishing Odom: %s", publish_tf_ ? "Yes" : "No", publish_odom_ ? "Yes" : "No");
  }

  ~ShelfbotORB3Node() {
    if (slam_system_) {
      RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Shutting down ORB-SLAM3...");
      slam_system_->Shutdown();
    }
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  // Add these to your class private members
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // TROUBLESHOOT: Individual callback for raw image messages
  void imageCallbackRaw(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    ++image_count_;
    static auto last_log = now();
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Raw image received: %d total, stamp: %d.%09d", image_count_, msg->header.stamp.sec, msg->header.stamp.nanosec);
    if ((now() - last_log).seconds() > 5.0) {  // Log rate every 5s
      RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Raw image received: %d total, rate ~%.1f Hz, stamp: %d.%09d", image_count_, image_count_ / 5.0, msg->header.stamp.sec, msg->header.stamp.nanosec);
      image_count_ = 0;
      last_log = now();
    }
  }

  // TROUBLESHOOT: Individual callback for raw camera_info messages
  void cameraInfoCallbackRaw(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    ++camera_info_count_;
    static auto last_log = now();
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Raw camera_info received: %d total, stamp: %d.%09d", camera_info_count_, msg->header.stamp.sec, msg->header.stamp.nanosec);
    if ((now() - last_log).seconds() > 5.0) {  // Log rate every 5s
      RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Raw camera_info received: %d total, rate ~%.1f Hz, stamp: %d.%09d", camera_info_count_, camera_info_count_ / 5.0, msg->header.stamp.sec, msg->header.stamp.nanosec);
      camera_info_count_ = 0;
      last_log = now();
    }
  }

  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: SYNC CALLBACK TRIGGERED! Image stamp: %d.%09d, Info stamp: %d.%09d, Delta: %.6f s",
                image_msg->header.stamp.sec, image_msg->header.stamp.nanosec, camera_info_msg->header.stamp.sec, camera_info_msg->header.stamp.nanosec,
                fabs(rclcpp::Time(image_msg->header.stamp).seconds() - rclcpp::Time(camera_info_msg->header.stamp).seconds()));

    if (!slam_initialized_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "shelfbot_slam_orb3: SLAM not initialized yet");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "shelfbot_slam_orb3: cv_bridge exception: %s", e.what());
        return;
    }

    const cv::Mat& gray_image = cv_ptr->image;
    if (gray_image.empty()) {
        RCLCPP_WARN(get_logger(), "shelfbot_slam_orb3: Received empty grayscale image");
        return;
    }

    const double timestamp = rclcpp::Time(image_msg->header.stamp).seconds();
    const Sophus::SE3f pose_se3 = slam_system_->TrackMonocular(gray_image, timestamp);
    const int tracking_state = slam_system_->GetTrackingState();
    const bool tracking_successful = (tracking_state == static_cast<int>(ORB_SLAM3::Tracking::OK));

    // TROUBLESHOOT: Log ORB features using standalone ORB extractor
    auto orb_extractor = cv::ORB::create(1200, 1.2f, 8, 20, 0, 2, cv::ORB::HARRIS_SCORE, 20);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb_extractor->detectAndCompute(gray_image, cv::Mat(), keypoints, descriptors);
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: TrackMonocular called, state: %d, successful: %s, features: %zu", tracking_state, tracking_successful ? "Yes" : "No", keypoints.size());

    // === FEATURE LOGGING ADDED ===
    RCLCPP_INFO(get_logger(), "shelfbot_slam_orb3: Descriptor matrix size: %d x %d (rows x cols)", descriptors.rows, descriptors.cols);
    for (size_t i = 0; i < std::min<size_t>(keypoints.size(), 10); ++i) {
      const auto & kp = keypoints[i];
      RCLCPP_INFO(get_logger(), "  KP[%zu]: pt=(%.2f, %.2f), size=%.2f, angle=%.2f, response=%.4f", i, kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response);
    }
    // === END FEATURE LOGGING ===

    const auto current_time = this->now();
    if (tracking_successful) {
        last_tracking_time_ = current_time;
        last_pose_valid_ = true;
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        const Eigen::Matrix4f eigen_Tcw = pose_se3.matrix();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                Tcw.at<float>(r, c) = eigen_Tcw(r, c);
            }
        }
        processSuccessfulTracking(Tcw, image_msg->header.stamp, camera_info_msg);
    } else {
        const double time_since_last_track = (current_time - last_tracking_time_).seconds();
        if (time_since_last_track > tracking_lost_timeout_) {
            if (last_pose_valid_) {
                RCLCPP_WARN(get_logger(), "shelfbot_slam_orb3: SLAM tracking lost for %.1f seconds (state: %d)", 
                            time_since_last_track, tracking_state);
                last_pose_valid_ = false;
            }
        }
    }
  }

  /**
  * @brief Processes the pose output from a successful ORB-SLAM3 tracking update.
  *
  * This function takes the raw pose matrix from ORB-SLAM3, converts it into
  * a standard tf2::Transform object, and then passes it to the dedicated
  * TF publishing function. It no longer contains any direct publishing logic.
  *
  * @param Tcw The 4x4 pose matrix (Cv::Mat) from ORB-SLAM3.
  * @param stamp The timestamp for the pose.
  * @param camera_info (unused) The associated camera info message.
  */
  void processSuccessfulTracking(const cv::Mat& Tcw, const builtin_interfaces::msg::Time& stamp, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    // This argument is unused in the new logic but kept for function signature consistency.
    (void)camera_info;

    // 1. Convert camera pose in world (Tcw) to world pose in camera (Twc = Tcw^-1)
    // This calculation remains the same.
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc * tcw;

    // 2. Convert the OpenCV matrix components into tf2 data types.
    tf2::Matrix3x3 tf_rotation(
        Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
        Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
        Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2)
    );
    
    tf2::Quaternion tf_quaternion;
    tf_rotation.getRotation(tf_quaternion);
    tf_quaternion.normalize(); // Ensure the quaternion is valid.

    tf2::Vector3 tf_translation(
        twc.at<float>(0), twc.at<float>(1), twc.at<float>(2)
    );

    // 3. Combine the components into a single tf2::Transform object.
    // This is the clean representation of the camera's pose in the map frame.
    tf2::Transform camera_pose_in_map(tf_quaternion, tf_translation);

    // 4. Delegate the complex TF publishing logic to the new, specialized function.
    // This replaces all previous calls to publishTransform, publishOdometry, and the
    // incorrect identity map->odom publisher.
    if (publish_tf_) {
        publishMapToOdomTransform(camera_pose_in_map, stamp);
    }

    // 5. Keep the logging for debugging purposes.
    static int pose_count = 0;
    if (++pose_count % 30 == 1) { // Log every ~1 second at 30fps
        RCLCPP_INFO(get_logger(),
            "SLAM pose (camera in map): [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
            tf_translation.x(), tf_translation.y(), tf_translation.z(),
            tf_quaternion.x(), tf_quaternion.y(), tf_quaternion.z(), tf_quaternion.w());
    }
  }

  /**
  * @brief Calculates and publishes the transform from the map frame to the odom frame.
  *
  * This is the primary role of a SLAM system in the ROS 2 navigation stack. It corrects
  * the drift of the odometry frame (`odom` -> `base_link`) by providing its pose
  * within the globally consistent `map` frame.
  *
  * The calculation is: T_map_odom = (T_map_camera * T_camera_base) * (T_odom_base)^-1
  * which simplifies to: T_map_odom = T_map_base * (T_odom_base)^-1
  *
  * @param camera_pose_in_map The pose of the camera in the map frame, from ORB-SLAM3.
  * @param stamp The timestamp for the transform.
  */
  void publishMapToOdomTransform(const tf2::Transform& camera_pose_in_map, const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::TransformStamped odom_to_base_footprint_tf;
    geometry_msgs::msg::TransformStamped base_footprint_to_base_link_tf;
    geometry_msgs::msg::TransformStamped base_to_camera_link_tf;
    
    try {
        RCLCPP_DEBUG(get_logger(), "Attempting to lookup transforms: odom->base_footprint->base_link->camera_link");
        
        // Look up the transform chain: odom->base_footprint->base_link->camera_link
        odom_to_base_footprint_tf = tf_buffer_->lookupTransform(
            odom_frame_, "base_footprint", tf2::TimePointZero);
            
        base_footprint_to_base_link_tf = tf_buffer_->lookupTransform(
            "base_footprint", base_link_frame_, tf2::TimePointZero);

        base_to_camera_link_tf = tf_buffer_->lookupTransform(
            base_link_frame_, camera_frame_, tf2::TimePointZero);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not get required transforms: %s", ex.what());
        return;
    }

    // Convert geometry_msgs::Transform to tf2::Transform and chain them
    tf2::Transform odom_to_base_footprint, base_footprint_to_base_link;
    tf2::fromMsg(odom_to_base_footprint_tf.transform, odom_to_base_footprint);
    tf2::fromMsg(base_footprint_to_base_link_tf.transform, base_footprint_to_base_link);
    
    // Combine to get odom->base_link
    tf2::Transform odom_to_base_link = odom_to_base_footprint * base_footprint_to_base_link;

    tf2::Transform base_to_camera_link;
    tf2::fromMsg(base_to_camera_link_tf.transform, base_to_camera_link);

    // Calculate the robot's base pose in the map frame
    tf2::Transform map_to_base_link = camera_pose_in_map * base_to_camera_link.inverse();

    // The map->odom transform is the difference between the SLAM pose and the odom pose
    tf2::Transform map_to_odom_tf = map_to_base_link * odom_to_base_link.inverse();

    // Publish the final transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    t.transform = tf2::toMsg(map_to_odom_tf);

    tf_broadcaster_->sendTransform(t);
    RCLCPP_DEBUG(get_logger(), "Published map -> odom TF");
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
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<ShelfbotORB3Node>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("shelfbot_slam_orb3: main"), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
