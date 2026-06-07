#include "shelfbot/four_wheel_drive_odometry.hpp"
#include "shelfbot/shelfbot_utils.hpp"  // pulls in log_zip.hpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace shelfbot {

FourWheelDriveOdometry::FourWheelDriveOdometry(
    std::shared_ptr<rclcpp::Node> node,
    const rclcpp::Clock::SharedPtr& clock,
    double wheel_separation,
    double wheel_radius)
  : node_(node),
    clock_(clock),
    wheel_separation_(wheel_separation),
    wheel_radius_(wheel_radius)
{
  odom_pub_     = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  pose_covariance_.fill(0.0);
  twist_covariance_.fill(0.0);

  // ── log_zip: odometry initialised ────────────────────────────────────────
  log_zip("ODO", "INIT", {{"sep", wheel_separation_}, {"rad", wheel_radius_}});

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

    // Indices: 0=FL, 1=FR, 2=BL, 3=BR
    double left_pos  = (wheel_positions[0] + wheel_positions[2]) * 0.5;   // FL + BL
    double right_pos = (wheel_positions[1] + wheel_positions[3]) * 0.5;   // FR + BR

    if (!initialized_) {
        prev_left_pos_  = left_pos;
        prev_right_pos_ = right_pos;
        initialized_ = true;
        // ── log_zip: first reading latched ───────────────────────────────────
        log_zip("ODO", "LATCH", {
            {"left", left_pos}, {"right", right_pos}
        });
        log_info("FourWheelDriveOdometry", "update",
                 "Odometry initialized with left/right wheel positions");
        return;
    }

    double left_diff  = (left_pos  - prev_left_pos_)  * wheel_radius_;
    double right_diff = (right_pos - prev_right_pos_) * wheel_radius_;

    // Forward displacement = average of left and right motion
    double fwd_dist = (left_diff + right_diff) * 0.5;
    // Rotational displacement = (right - left) / wheel_separation
    double rotation = (right_diff - left_diff) / wheel_separation_;

    theta_ += rotation;
    x_     += fwd_dist * std::cos(theta_);
    y_     += fwd_dist * std::sin(theta_);

    // ── log_zip: pose update (core odometry line) ────────────────────────────
    log_zip("ODO", "UPD", {
        {"x",  x_}, {"y", y_}, {"th", theta_},
        {"df", fwd_dist}, {"dr", rotation}, {"dt", period_sec}
    });

    prev_left_pos_  = left_pos;
    prev_right_pos_ = right_pos;

    auto stamp = clock_->now();

    auto odom_msg           = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp    = stamp;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id  = "base_footprint";
    odom_msg->pose.pose       = calculate_pose();
    odom_msg->pose.covariance = calculate_pose_covariance();
    odom_msg->twist.twist     = calculate_twist(wheel_positions, period);
    odom_msg->twist.covariance = calculate_twist_covariance();

    odom_pub_->publish(std::move(odom_msg));
    broadcast_tf(stamp);
}

void FourWheelDriveOdometry::broadcast_tf(const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp    = stamp;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id  = "base_footprint";

  odom_tf.transform.translation.x = x_;
  odom_tf.transform.translation.y = y_;
  odom_tf.transform.translation.z = 0.0;

  double final_theta = theta_ - 1.57079632679;
  odom_tf.transform.rotation =
      tf2::toMsg(tf2::Quaternion(
        0, 0,
        std::sin(final_theta / 2.0),
        std::cos(final_theta / 2.0)));

  tf_broadcaster_->sendTransform(odom_tf);

  // ── log_zip: TF broadcast (sampled — every call but cheap) ───────────────
  log_zip("ODO", "TF", {
      {"x", x_}, {"y", y_}, {"th_adj", final_theta}
  });
}

nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = clock_->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_footprint";
  odom.pose.pose       = calculate_pose();
  return odom;
}

geometry_msgs::msg::Pose FourWheelDriveOdometry::calculate_pose() const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x_;
  pose.position.y = y_;
  pose.position.z = 0.0;

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

geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period)
{
  double dt = period.seconds();
  if (dt < 1e-9) { dt = 1e-9; }

  double left_pos  = (wheel_positions[0] + wheel_positions[2]) * 0.5;
  double right_pos = (wheel_positions[1] + wheel_positions[3]) * 0.5;

  double left_vel  = (left_pos  - prev_left_pos_)  * wheel_radius_ / dt;
  double right_vel = (right_pos - prev_right_pos_) * wheel_radius_ / dt;

  geometry_msgs::msg::Twist twist;
  twist.linear.x  = (left_vel + right_vel) * 0.5;
  twist.angular.z = (right_vel - left_vel) / wheel_separation_;

  // ── log_zip: twist (linear + angular) ────────────────────────────────────
  log_zip("ODO", "TWS", {{"vx", twist.linear.x}, {"wz", twist.angular.z}});

  return twist;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance()
{
  pose_covariance_.fill(0.0);
  pose_covariance_[0]  = 0.1;
  pose_covariance_[7]  = 0.1;
  pose_covariance_[35] = 0.2;
  return pose_covariance_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance()
{
  twist_covariance_.fill(0.0);
  twist_covariance_[0]  = 0.1;
  twist_covariance_[35] = 0.2;
  return twist_covariance_;
}

}  // namespace shelfbot