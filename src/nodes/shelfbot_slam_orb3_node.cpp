#include "shelfbot/shelfbot_slam_orb3_node.hpp"

ShelfbotORB3Node::ShelfbotORB3Node(const rclcpp::NodeOptions & opts) : Node("shelfbot_slam_orb3_node", opts), slam_initialized_(false), last_pose_valid_(false), image_count_(0), camera_info_count_(0) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

  if (voc_file_.empty() || settings_file_.empty()) {
    shelfbot::log_error("shelfbot_slam_orb3_node", "PARAM_ERROR", "shelfbot_slam_orb3: voc_file and settings_file parameters are required!");
    rclcpp::shutdown();
    return;
  }

  try {
    slam_system_ = std::make_shared<ORB_SLAM3::System>(
      voc_file_, settings_file_,
      ORB_SLAM3::System::MONOCULAR,
      false
    );
    slam_initialized_ = true;
    shelfbot::log_info("shelfbot_slam_orb3_node", "INIT_SUCCESS", "shelfbot_slam_orb3: ORB-SLAM3 system initialized successfully");
  } catch (const std::exception& e) {
    shelfbot::log_error("shelfbot_slam_orb3_node", "INIT_FAIL", std::string("shelfbot_slam_orb3: Failed to initialize ORB-SLAM3: ") + e.what());
    rclcpp::shutdown();
    return;
  }

  image_sub_raw_ = this->create_subscription<sensor_msgs::msg::Image>( camera_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&ShelfbotORB3Node::imageCallbackRaw, this, std::placeholders::_1));
  camera_info_sub_raw_ = this->create_subscription<sensor_msgs::msg::CameraInfo>( camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&ShelfbotORB3Node::cameraInfoCallbackRaw, this, std::placeholders::_1));

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
    
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), image_sub_, camera_info_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_->registerCallback(std::bind(&ShelfbotORB3Node::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  shelfbot::log_info("shelfbot_slam_orb3_node", "SYNC_REG", "shelfbot_slam_orb3: Sync registered with queue=20, slop=0.1s");

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
    
  if (publish_odom_) {
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("slam_odom", rclcpp::QoS(10));
  }

  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(1));

  last_tracking_time_ = now();
    
  shelfbot::log_info("shelfbot_slam_orb3_node", "SLAM_INIT", std::string("shelfbot_slam_orb3: ShelfBot SLAM initialized. Image: [") + camera_topic_ + "], CameraInfo: [" + camera_info_topic_ + "]");
  shelfbot::log_info("shelfbot_slam_orb3_node", "PUB_CONFIG", std::string("shelfbot_slam_orb3: Publishing TF: ") + (publish_tf_ ? "Yes" : "No") + ", Publishing Odom: " + (publish_odom_ ? "Yes" : "No"));
}

ShelfbotORB3Node::~ShelfbotORB3Node() {
  if (slam_system_) {
    shelfbot::log_info("shelfbot_slam_orb3_node", "SHUTDOWN", "shelfbot_slam_orb3: Shutting down ORB-SLAM3...");
    slam_system_->Shutdown();
  }
}

void ShelfbotORB3Node::imageCallbackRaw(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  ++image_count_;
  static auto last_log = now();
  shelfbot::log_info("shelfbot_slam_orb3_node", "RAW_IMAGE", std::string("shelfbot_slam_orb3: Raw image received: ") + std::to_string(image_count_) + " total, stamp: " + std::to_string(msg->header.stamp.sec) + "." + std::to_string(msg->header.stamp.nanosec));
  if ((now() - last_log).seconds() > 5.0) {  // Log rate every 5s
    shelfbot::log_info("shelfbot_slam_orb3_node", "RAW_IMAGE_RATE", std::string("shelfbot_slam_orb3: Raw image received: ") + std::to_string(image_count_) + " total, rate ~" + std::to_string(image_count_ / 5.0) + " Hz, stamp: " + std::to_string(msg->header.stamp.sec) + "." + std::to_string(msg->header.stamp.nanosec));
    image_count_ = 0;
    last_log = now();
  }
}

void ShelfbotORB3Node::cameraInfoCallbackRaw(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  ++camera_info_count_;
  static auto last_log = now();
  shelfbot::log_info("shelfbot_slam_orb3_node", "RAW_CAMINFO", std::string("shelfbot_slam_orb3: Raw camera_info received: ") + std::to_string(camera_info_count_) + " total, stamp: " + std::to_string(msg->header.stamp.sec) + "." + std::to_string(msg->header.stamp.nanosec));
  if ((now() - last_log).seconds() > 5.0) {  // Log rate every 5s
    shelfbot::log_info("shelfbot_slam_orb3_node", "RAW_CAMINFO_RATE", std::string("shelfbot_slam_orb3: Raw camera_info received: ") + std::to_string(camera_info_count_) + " total, rate ~" + std::to_string(camera_info_count_ / 5.0) + " Hz, stamp: " + std::to_string(msg->header.stamp.sec) + "." + std::to_string(msg->header.stamp.nanosec));
    camera_info_count_ = 0;
    last_log = now();
  }
}

void ShelfbotORB3Node::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
  shelfbot::log_info("shelfbot_slam_orb3_node", "SYNC_TRIGGER", std::string("shelfbot_slam_orb3: SYNC CALLBACK TRIGGERED! Image stamp: ") + std::to_string(image_msg->header.stamp.sec) + "." + std::to_string(image_msg->header.stamp.nanosec) + ", Info stamp: " + std::to_string(camera_info_msg->header.stamp.sec) + "." + std::to_string(camera_info_msg->header.stamp.nanosec) + ", Delta: " + std::to_string(fabs(rclcpp::Time(image_msg->header.stamp).seconds() - rclcpp::Time(camera_info_msg->header.stamp).seconds())) + " s");

  if (!slam_initialized_) {
      shelfbot::log_warn("shelfbot_slam_orb3_node", "SLAM_NOT_READY", "shelfbot_slam_orb3: SLAM not initialized yet");
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception& e) {
      shelfbot::log_error("shelfbot_slam_orb3_node", "CV_BRIDGE_ERR", std::string("shelfbot_slam_orb3: cv_bridge exception: ") + e.what());
      return;
  }

  const cv::Mat& gray_image = cv_ptr->image;
  if (gray_image.empty()) {
      shelfbot::log_warn("shelfbot_slam_orb3_node", "EMPTY_IMAGE", "shelfbot_slam_orb3: Received empty grayscale image");
      return;
  }

  cv::Scalar mean, stddev;
  cv::meanStdDev(gray_image, mean, stddev);
  shelfbot::log_info("shelfbot_slam_orb3_node", "IMAGE_STATS", std::string("shelfbot_slam_orb3: Image stats: mean=") + std::to_string(mean[0]) + ", stddev=" + std::to_string(stddev[0]) + ", size=" + std::to_string(gray_image.cols) + "x" + std::to_string(gray_image.rows));

  const double timestamp = rclcpp::Time(image_msg->header.stamp).seconds();
  double timestamp_diff = timestamp - last_timestamp_;
  shelfbot::log_info("shelfbot_slam_orb3_node", "TIMESTAMP_DIFF", std::string("shelfbot_slam_orb3: Timestamp diff from last: ") + std::to_string(timestamp_diff) + " s");
  last_timestamp_ = timestamp;

  double norm_diff = 0.0;
  if (!last_gray_image_.empty()) {
    norm_diff = cv::norm(gray_image, last_gray_image_, cv::NORM_L2) / (gray_image.rows * gray_image.cols);
    shelfbot::log_info("shelfbot_slam_orb3_node", "IMAGE_DIFF", std::string("shelfbot_slam_orb3: Image diff norm from last: ") + std::to_string(norm_diff) + " (low = no motion)");
  }
  last_gray_image_ = gray_image.clone();

  const Sophus::SE3f pose_se3 = slam_system_->TrackMonocular(gray_image, timestamp);
  const int tracking_state = slam_system_->GetTrackingState();
  const bool tracking_successful = (tracking_state == static_cast<int>(ORB_SLAM3::Tracking::OK));

  shelfbot::log_info("shelfbot_slam_orb3_node", "POSE_FINITE", std::string("shelfbot_slam_orb3: Pose matrix finite: ") + (pose_se3.matrix().allFinite() ? "Yes" : "No"));
  for (int i = 0; i < 4; ++i) {
    shelfbot::log_info("shelfbot_slam_orb3_node", "POSE_ROW", std::string("shelfbot_slam_orb3: Pose row ") + std::to_string(i) + ": " + std::to_string(pose_se3.matrix()(i,0)) + " " + std::to_string(pose_se3.matrix()(i,1)) + " " + std::to_string(pose_se3.matrix()(i,2)) + " " + std::to_string(pose_se3.matrix()(i,3)));
  }

  auto orb_extractor = cv::ORB::create(1200, 1.2f, 8, 20, 0, 2, cv::ORB::HARRIS_SCORE, 20);
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  orb_extractor->detectAndCompute(gray_image, cv::Mat(), keypoints, descriptors);
  shelfbot::log_info("shelfbot_slam_orb3_node", "TRACK_MONO", std::string("shelfbot_slam_orb3: TrackMonocular called, state: ") + std::to_string(tracking_state) + ", successful: " + (tracking_successful ? "Yes" : "No") + ", features: " + std::to_string(keypoints.size()));

  shelfbot::log_info("shelfbot_slam_orb3_node", "DESCRIPTOR_SIZE", std::string("shelfbot_slam_orb3: Descriptor matrix size: ") + std::to_string(descriptors.rows) + " x " + std::to_string(descriptors.cols) + " (rows x cols)");
  for (size_t i = 0; i < std::min<size_t>(keypoints.size(), 10); ++i) {
    const auto & kp = keypoints[i];
    shelfbot::log_info("shelfbot_slam_orb3_node", "KEYPOINT", std::string("KP[") + std::to_string(i) + "]: pt=(" + std::to_string(kp.pt.x) + ", " + std::to_string(kp.pt.y) + "), size=" + std::to_string(kp.size) + ", angle=" + std::to_string(kp.angle) + ", response=" + std::to_string(kp.response));
  }

  const auto current_time = this->now();
  if (tracking_successful) {
      last_tracking_time_ = current_time;
      last_pose_valid_ = true;
      Eigen::Matrix4f eigen_Tcw = pose_se3.matrix();
      cv::Mat Tcw(4, 4, CV_32F, eigen_Tcw.data());
      processSuccessfulTracking(Tcw, image_msg->header.stamp, camera_info_msg);
  } else {
      const double time_since_last_track = (current_time - last_tracking_time_).seconds();
      shelfbot::log_info("shelfbot_slam_orb3_node", "TIME_SINCE_TRACK", std::string("shelfbot_slam_orb3: Time since last track: ") + std::to_string(time_since_last_track) + " s (timeout: " + std::to_string(tracking_lost_timeout_) + ")");
      if (time_since_last_track > tracking_lost_timeout_) {
          if (last_pose_valid_) {
              shelfbot::log_warn("shelfbot_slam_orb3_node", "TRACKING_LOST", std::string("shelfbot_slam_orb3: SLAM tracking lost for ") + std::to_string(time_since_last_track) + " seconds (state: " + std::to_string(tracking_state) + ")");
              last_pose_valid_ = false;
          }
      }
      if (tracking_state == 1) {
          shelfbot::log_debug("shelfbot_slam_orb3_node", "NOT_INITIALIZED", "shelfbot_slam_orb3: NOT_INITIALIZED - Insufficient parallax or inliers. Move camera with translation and rotation.");
      } else if (tracking_state == 3) {
          shelfbot::log_debug("shelfbot_slam_orb3_node", "LOST", "shelfbot_slam_orb3: LOST - Relocalizing...");
      } else if (tracking_state == 0) {
          shelfbot::log_debug("shelfbot_slam_orb3_node", "NO_IMAGES", "shelfbot_slam_orb3: NO_IMAGES_YET - Waiting for first image.");
      } else if (tracking_state == 4) {
          shelfbot::log_debug("shelfbot_slam_orb3_node", "SYS_FAIL", "shelfbot_slam_orb3: NOT_INITIALIZED (system failure).");
      }
  }
}

void ShelfbotORB3Node::processSuccessfulTracking(const cv::Mat& Tcw, const builtin_interfaces::msg::Time& stamp, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
  (void)camera_info;

  cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
  cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
  cv::Mat Rwc = Rcw.t();
  cv::Mat twc = -Rwc * tcw;

  tf2::Matrix3x3 tf_rotation(
      Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
      Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
      Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2)
  );
  
  tf2::Quaternion tf_quaternion;
  tf_rotation.getRotation(tf_quaternion);
  tf_quaternion.normalize();

  tf2::Vector3 tf_translation(
      twc.at<float>(0), twc.at<float>(1), twc.at<float>(2)
  );

  tf2::Transform cv_pose_in_map(tf_quaternion, tf_translation);
  tf2::Transform camera_pose_in_map = convertFromCvToRos(cv_pose_in_map);

  if (!tf_buffer_->canTransform(base_link_frame_, camera_frame_, stamp, rclcpp::Duration::from_seconds(1.0))) {
      shelfbot::log_warn("shelfbot_slam_orb3_node", "TF_NOT_READY", "shelfbot_slam_orb3: TF chain not ready, skipping publish");
      return;
  }

  geometry_msgs::msg::TransformStamped base_to_camera_link_tf;
  try {
      base_to_camera_link_tf = tf_buffer_->lookupTransform(
          base_link_frame_, camera_frame_, stamp, rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
      shelfbot::log_warn("shelfbot_slam_orb3_node", "TF_LOOKUP_FAIL", std::string("shelfbot_slam_orb3: TF lookup failed: ") + ex.what());
      return;
  }
  tf2::Transform base_to_camera_link;
  tf2::fromMsg(base_to_camera_link_tf.transform, base_to_camera_link);
  tf2::Transform map_to_base_link = camera_pose_in_map * base_to_camera_link.inverse();

  if (publish_tf_) {
      publishMapToOdomTransform(camera_pose_in_map, stamp);
  }

  publishSlamOdom(camera_pose_in_map, stamp);

  static int pose_count = 0;
  if (++pose_count % 30 == 1) {
      const tf2::Vector3 t = camera_pose_in_map.getOrigin();
      const tf2::Quaternion q = camera_pose_in_map.getRotation();
      shelfbot::log_info("shelfbot_slam_orb3_node", "SLAM_POSE", std::string("shelfbot_slam_orb3: SLAM pose (camera in map, ROS frame): [") + std::to_string(t.x()) + ", " + std::to_string(t.y()) + ", " + std::to_string(t.z()) + "] [" + std::to_string(q.x()) + ", " + std::to_string(q.y()) + ", " + std::to_string(q.z()) + ", " + std::to_string(q.w()) + "]");
  }
}

void ShelfbotORB3Node::publishMapToOdomTransform(const tf2::Transform& camera_pose_in_map, const builtin_interfaces::msg::Time& stamp) {
  if (!tf_buffer_->canTransform(odom_frame_, "base_footprint", stamp, rclcpp::Duration::from_seconds(1.0)) ||
      !tf_buffer_->canTransform("base_footprint", base_link_frame_, stamp, rclcpp::Duration::from_seconds(1.0)) ||
      !tf_buffer_->canTransform(base_link_frame_, camera_frame_, stamp, rclcpp::Duration::from_seconds(1.0))) {
      shelfbot::log_warn("shelfbot_slam_orb3_node", "TF_CHAIN_NOT_READY", "shelfbot_slam_orb3: TF chain not ready, skipping TF publish");
      return;
  }

  geometry_msgs::msg::TransformStamped odom_to_base_footprint_tf;
  geometry_msgs::msg::TransformStamped base_footprint_to_base_link_tf;
  geometry_msgs::msg::TransformStamped base_to_camera_link_tf;
  
  try {
      shelfbot::log_debug("shelfbot_slam_orb3_node", "TF_LOOKUP_START", "shelfbot_slam_orb3: Attempting to lookup transforms: odom->base_footprint->base_link->camera_link");
      
      odom_to_base_footprint_tf = tf_buffer_->lookupTransform(
          odom_frame_, "base_footprint", stamp, rclcpp::Duration::from_seconds(1.0));
      
      base_footprint_to_base_link_tf = tf_buffer_->lookupTransform(
          "base_footprint", base_link_frame_, stamp, rclcpp::Duration::from_seconds(1.0));

      base_to_camera_link_tf = tf_buffer_->lookupTransform(
          base_link_frame_, camera_frame_, stamp, rclcpp::Duration::from_seconds(1.0));

  } catch (const tf2::TransformException & ex) {
      shelfbot::log_warn("shelfbot_slam_orb3_node", "TF_LOOKUP_FAIL", std::string("shelfbot_slam_orb3: Could not get required transforms: ") + ex.what());
      return;
  }

  tf2::Transform odom_to_base_footprint, base_footprint_to_base_link;
  tf2::fromMsg(odom_to_base_footprint_tf.transform, odom_to_base_footprint);
  tf2::fromMsg(base_footprint_to_base_link_tf.transform, base_footprint_to_base_link);
  
  tf2::Transform odom_to_base_link = odom_to_base_footprint * base_footprint_to_base_link;

  tf2::Transform base_to_camera_link;
  tf2::fromMsg(base_to_camera_link_tf.transform, base_to_camera_link);

  tf2::Transform map_to_base_link = camera_pose_in_map * base_to_camera_link.inverse();

  tf2::Transform map_to_odom_tf = map_to_base_link * odom_to_base_link.inverse();

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = map_frame_;
  t.child_frame_id = odom_frame_;
  t.transform = tf2::toMsg(map_to_odom_tf);

  tf_broadcaster_->sendTransform(t);
  shelfbot::log_debug("shelfbot_slam_orb3_node", "TF_PUBLISH_SUCCESS", "shelfbot_slam_orb3: Published map -> odom TF");
}

void ShelfbotORB3Node::publishSlamOdom(const tf2::Transform& camera_pose_in_map, const builtin_interfaces::msg::Time& stamp) {
  if (!publish_odom_) return;
  // Use camera_pose_in_map as proxy for map_to_base_link (assume base_to_camera identity for initial)
  tf2::Transform map_to_base_link = camera_pose_in_map; // Fix: no TF dependency
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = map_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  tf2::toMsg(map_to_base_link, odom_msg.pose.pose);
  std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
  odom_msg.pose.covariance[0] = odom_msg.pose.covariance[7] = odom_msg.pose.covariance[14] = 0.01;
  odom_msg.pose.covariance[21] = odom_msg.pose.covariance[28] = odom_msg.pose.covariance[35] = 0.01;
  odom_publisher_->publish(odom_msg);
  shelfbot::log_debug("shelfbot_slam_orb3_node", "ODOM_PUBLISH", "Published /slam_odom");
}

tf2::Transform ShelfbotORB3Node::convertFromCvToRos(const tf2::Transform& cv_tf) {
  tf2::Matrix3x3 R_cv_to_ros( 0,  0,  1,
                             -1,  0,  0,
                              0, -1,  0);
  tf2::Transform ros_tf;
  ros_tf.setOrigin(cv_tf.getOrigin());
  ros_tf.setBasis(R_cv_to_ros * cv_tf.getBasis());
  return ros_tf;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<ShelfbotORB3Node>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    shelfbot::log_error("shelfbot_slam_orb3_node", "MAIN_EXCEPTION", std::string("shelfbot_slam_orb3: Exception in main: ") + e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
