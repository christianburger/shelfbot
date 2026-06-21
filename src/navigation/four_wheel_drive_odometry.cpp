#include "shelfbot/four_wheel_drive_odometry.hpp"
#include "shelfbot/shelfbot_utils.hpp"  // pulls in log_zip.hpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace shelfbot {

    FourWheelDriveOdometry::FourWheelDriveOdometry(
        std::shared_ptr<rclcpp::Node> node, const rclcpp::Clock::SharedPtr& clock, double wheel_separation, double wheel_radius, bool publish_tf) : node_(node), clock_(clock), wheel_separation_(wheel_separation), wheel_radius_(wheel_radius), publish_tf_(publish_tf) { // removed: publish_tf_ = node_->declare_parameter<bool>("odometry_publish_tf", true);
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("wheel_odom_raw", 10);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
            log_info("FourWheelDriveOdometry", "Constructor",
                     "TF broadcasting ENABLED (raw odometry owns odom→base_footprint)");
        } else {
            log_info("FourWheelDriveOdometry", "Constructor",
                     "TF broadcasting DISABLED (EKF owns odom→base_footprint)");
        }
        // rest unchanged

  // ── publish_tf parameter ──────────────────────────────────────────────────
  // When the EKF (robot_localization) is running, the EKF owns the
  // odom→base_footprint TF.  Two nodes broadcasting the same TF edge
  // causes "extrapolation into the future" errors in the controller because
  // the TF buffer receives two conflicting entry streams.
  //
  // Set publish_tf to false in the hardware interface parameters (or via
  // four_wheel_drive_controller.yaml) when the EKF node is in the launch.
  // The default is TRUE so that the system works without an EKF (e.g. during
  // calibration or debugging without robot_localization installed).
  publish_tf_ = node_->declare_parameter<bool>("odometry_publish_tf", true);

  // ── Raw odometry topic ────────────────────────────────────────────────────
  // Publish to /wheel_odom_raw (not /odom) so the EKF can consume the raw
  // wheel data while the EKF's filtered output owns the /odom topic.
  // The launch file remaps the EKF's odometry/filtered → /odom so all
  // downstream consumers (Nav2, slam_toolbox) still see a single /odom.
  //
  // If no EKF is present in the launch, remap this topic back to /odom via:
  //   remappings=[('wheel_odom_raw', 'odom')]
  // in the ros2_control_node entry of the launch file, or simply set
  // odometry_publish_tf=true and add back the /odom publisher name below.
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("wheel_odom_raw", 10);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    log_info("FourWheelDriveOdometry", "Constructor",
             "TF broadcasting ENABLED (raw odometry owns odom→base_footprint)");
  } else {
    log_info("FourWheelDriveOdometry", "Constructor",
             "TF broadcasting DISABLED (EKF owns odom→base_footprint)");
  }

  pose_covariance_.fill(0.0);
  twist_covariance_.fill(0.0);

  log_zip("ODO", "INIT", {{"sep", wheel_separation_}, {"rad", wheel_radius_},
                           {"tf",  publish_tf_ ? 1.0 : 0.0}});

  log_info("FourWheelDriveOdometry", "Constructor",
           "Initialized with separation: " + std::to_string(wheel_separation_) +
           ", radius: " + std::to_string(wheel_radius_));
}

void FourWheelDriveOdometry::update(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period)
{
    double period_sec = period.seconds();

    if (period_sec <= 0.0) {
        log_zip("ODO", "ERR", {{"dt", period_sec}, {"code", -1}});
        log_warn("FourWheelDriveOdometry", "update",
                "Invalid period (<= 0): " + std::to_string(period_sec) + "s - skipping update");
        return;
    }
    if (period_sec > 1.0) {
        log_zip("ODO", "WARN", {{"dt", period_sec}, {"overload", 1}});
        log_warn("FourWheelDriveOdometry", "update",
                "Unusually long period: " + std::to_string(period_sec) + "s");
    }
    if (wheel_positions.size() < 4) {
        log_zip("ODO", "ERR", {{"wct", (double)wheel_positions.size()}, {"code", -2}});
        log_error("FourWheelDriveOdometry", "update",
                "Insufficient wheel positions: " + std::to_string(wheel_positions.size()));
        return;
    }

    // ── LEFT/RIGHT SIDE AVERAGING ──────────────────────────────────────────
    // Skid-steer model: average FL+BL for left, FR+BR for right.
    // [0]=FL, [1]=FR, [2]=BL, [3]=BR (all forward-positive after read() flip)
    double left_pos  = (wheel_positions[0] + wheel_positions[2]) * 0.5;
    double right_pos = (wheel_positions[1] + wheel_positions[3]) * 0.5;

    if (!initialized_) {
        prev_left_pos_  = left_pos;
        prev_right_pos_ = right_pos;
        initialized_ = true;
        log_zip("ODO", "LATCH", {{"left", left_pos}, {"right", right_pos}});
        log_info("FourWheelDriveOdometry", "update",
                 "Odometry initialized with left/right wheel positions");
        return;
    }

    double left_diff  = (left_pos  - prev_left_pos_)  * wheel_radius_;
    double right_diff = (right_pos - prev_right_pos_) * wheel_radius_;

    prev_left_pos_  = left_pos;
    prev_right_pos_ = right_pos;

    double fwd_dist = (left_diff + right_diff) * 0.5;
    double rotation = (right_diff - left_diff) / wheel_separation_;

    theta_ += rotation;
    x_     += fwd_dist * std::cos(theta_);
    y_     += fwd_dist * std::sin(theta_);

    log_zip("ODO", "UPD", {
        {"x",  x_}, {"y", y_}, {"th", theta_},
        {"df", fwd_dist}, {"dr", rotation}, {"dt", period_sec}
    });

    auto stamp = clock_->now();

    auto odom_msg           = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp    = stamp;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id  = "base_footprint";
    odom_msg->pose.pose       = calculate_pose();
    odom_msg->pose.covariance = calculate_pose_covariance();
    odom_msg->twist.twist     = calculate_twist(left_diff, right_diff, period_sec);
    odom_msg->twist.covariance = calculate_twist_covariance();

    odom_pub_->publish(std::move(odom_msg));

    // Only broadcast TF when the EKF is not running.
    // When publish_tf_=false the EKF owns this TF edge.
    if (publish_tf_) {
        broadcast_tf(stamp);
    }
}
    void FourWheelDriveOdometry::broadcast_tf(const rclcpp::Time& stamp) {
    if (!tf_broadcaster_) return;

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp    = stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id  = "base_footprint";

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;

    // No yaw offset. The URDF joint_base_footprint now carries rpy="0 0 -pi/2"
    // which aligns base_link's physical forward (+Y) with base_footprint's +X.
    // The odometry integrates along odom +X at theta_=0, which is correct per
    // REP-103. Both the TF and the /wheel_odom_raw message now use the same theta_.
    odom_tf.transform.rotation =
        tf2::toMsg(tf2::Quaternion(
          0, 0,
          std::sin(theta_ / 2.0),
          std::cos(theta_ / 2.0)));

    tf_broadcaster_->sendTransform(odom_tf);

    log_zip("ODO", "TF", {{"x", x_}, {"y", y_}, {"th", theta_}});
}

nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = clock_->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_footprint";
  odom.pose.pose       = calculate_pose();
  return odom;
}

geometry_msgs::msg::Pose FourWheelDriveOdometry::calculate_pose() const {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x_;
  pose.position.y = y_;
  pose.position.z = 0.0;

  // Raw theta_ without yaw offset — keeps the pose consistent with the twist.
  double final_theta = theta_;
  pose.orientation = tf2::toMsg(
    tf2::Quaternion(
      0, 0,
      std::sin(final_theta / 2.0),
      std::cos(final_theta / 2.0)));

  log_info("FourWheelDriveOdometry", "CalculatePose",
           "X: " + std::to_string(x_) +
           " Y: " + std::to_string(y_) +
           " Theta: " + std::to_string(theta_));
  return pose;
}

geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist(double left_diff_m, double right_diff_m, double dt_s) {
  if (dt_s < 1e-9) { dt_s = 1e-9; }

  double left_vel  = left_diff_m  / dt_s;
  double right_vel = right_diff_m / dt_s;

  geometry_msgs::msg::Twist twist;
  twist.linear.x  = (left_vel + right_vel) * 0.5;
  twist.angular.z = (right_vel - left_vel) / wheel_separation_;

  log_zip("ODO", "TWS", {{"vx", twist.linear.x}, {"wz", twist.angular.z}});

  return twist;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance() {
  pose_covariance_.fill(0.0);
  pose_covariance_[0]  = 0.1;
  pose_covariance_[7]  = 0.1;
  pose_covariance_[35] = 0.2;
  return pose_covariance_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance() {
  twist_covariance_.fill(0.0);
  twist_covariance_[0]  = 0.1;
  twist_covariance_[35] = 0.2;
  return twist_covariance_;
}

}
