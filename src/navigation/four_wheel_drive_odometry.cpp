#include "shelfbot/four_wheel_drive_odometry.hpp"
#include "shelfbot/shelfbot_utils.hpp"
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
    wheel_radius_(wheel_radius) {

  // Always broadcast odom→base_footprint
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // Publish directly to /odom.
  // slam_toolbox consumes this (via the TF chain) and publishes the
  // correcting map→odom TF.  Nav2 consumes both /odom and /map.
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  pose_covariance_.fill(0.0);
  twist_covariance_.fill(0.0);

  log_zip("ODO", "INIT", {{"sep", wheel_separation_}, {"rad", wheel_radius_}});
  log_info("FourWheelDriveOdometry", "Constructor",
           "sep=" + std::to_string(wheel_separation_) +
           " rad=" + std::to_string(wheel_radius_));
}

// ─────────────────────────────────────────────────────────────────────────────
void FourWheelDriveOdometry::update(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period)
{
    const double period_sec = period.seconds();

    if (period_sec <= 0.0) {
        log_zip("ODO", "ERR", {{"dt", period_sec}, {"code", -1}});
        log_warn("FourWheelDriveOdometry", "update",
                 "Invalid period: " + std::to_string(period_sec) + "s — skipping");
        return;
    }
    if (period_sec > 1.0) {
        log_zip("ODO", "WARN", {{"dt", period_sec}});
        log_warn("FourWheelDriveOdometry", "update",
                 "Unusually long period: " + std::to_string(period_sec) + "s");
    }
    if (wheel_positions.size() < 4) {
        log_zip("ODO", "ERR", {{"wct", (double)wheel_positions.size()}, {"code", -2}});
        log_error("FourWheelDriveOdometry", "update",
                  "Need 4 wheel positions, got " +
                  std::to_string(wheel_positions.size()));
        return;
    }

    // Average each side.  [0]=FL [1]=FR [2]=BL [3]=BR
    // All values are forward-positive after the read()-side flip in the HW interface.
    const double left_pos  = (wheel_positions[0] + wheel_positions[2]) * 0.5;
    const double right_pos = (wheel_positions[1] + wheel_positions[3]) * 0.5;

    if (!initialized_) {
        prev_left_pos_  = left_pos;
        prev_right_pos_ = right_pos;
        initialized_ = true;
        log_zip("ODO", "LATCH", {{"l", left_pos}, {"r", right_pos}});
        return;
    }

    const double left_diff  = (left_pos  - prev_left_pos_)  * wheel_radius_;
    const double right_diff = (right_pos - prev_right_pos_) * wheel_radius_;

    prev_left_pos_  = left_pos;
    prev_right_pos_ = right_pos;

    const double fwd_dist = (left_diff + right_diff) * 0.5;
    const double rotation = (right_diff - left_diff) / wheel_separation_;

    theta_ += rotation;
    x_     += fwd_dist * std::cos(theta_);
    y_     += fwd_dist * std::sin(theta_);

    log_zip("ODO", "UPD", {
        {"x",  x_}, {"y",  y_}, {"th", theta_},
        {"df", fwd_dist}, {"dr", rotation}, {"dt", period_sec}
    });

    const auto stamp = clock_->now();

    auto msg              = std::make_unique<nav_msgs::msg::Odometry>();
    msg->header.stamp     = stamp;
    msg->header.frame_id  = "odom";
    msg->child_frame_id   = "base_footprint";
    msg->pose.pose        = calculate_pose();
    msg->pose.covariance  = calculate_pose_covariance();
    msg->twist.twist      = calculate_twist(left_diff, right_diff, period_sec);
    msg->twist.covariance = calculate_twist_covariance();

    odom_pub_->publish(std::move(msg));
    broadcast_tf(stamp);
}

// ─────────────────────────────────────────────────────────────────────────────
void FourWheelDriveOdometry::broadcast_tf(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    // No yaw correction applied here.
    // The URDF joint_base_footprint carries rpy="0 0 -pi/2", which aligns
    // base_link's physical forward (+Y) with base_footprint +X (REP-103).
    // theta_ = 0 correctly means the robot faces odom +X.
    tf.transform.rotation = tf2::toMsg(
        tf2::Quaternion(0, 0,
            std::sin(theta_ / 2.0),
            std::cos(theta_ / 2.0)));

    tf_broadcaster_->sendTransform(tf);
    log_zip("ODO", "TF", {{"x", x_}, {"y", y_}, {"th", theta_}});
}

// ─────────────────────────────────────────────────────────────────────────────
nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = clock_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";
    odom.pose.pose       = calculate_pose();
    return odom;
}

// ─────────────────────────────────────────────────────────────────────────────
geometry_msgs::msg::Pose FourWheelDriveOdometry::calculate_pose() const
{
    geometry_msgs::msg::Pose pose;
    pose.position.x  = x_;
    pose.position.y  = y_;
    pose.position.z  = 0.0;
    pose.orientation = tf2::toMsg(
        tf2::Quaternion(0, 0,
            std::sin(theta_ / 2.0),
            std::cos(theta_ / 2.0)));

    log_info("FourWheelDriveOdometry", "CalculatePose",
             "X: " + std::to_string(x_) +
             " Y: " + std::to_string(y_) +
             " Theta: " + std::to_string(theta_));
    return pose;
}

// ─────────────────────────────────────────────────────────────────────────────
geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist(
    double left_diff_m, double right_diff_m, double dt_s)
{
    if (dt_s < 1e-9) dt_s = 1e-9;

    const double left_vel  = left_diff_m  / dt_s;
    const double right_vel = right_diff_m / dt_s;

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = (left_vel + right_vel) * 0.5;
    twist.angular.z = (right_vel - left_vel) / wheel_separation_;

    log_zip("ODO", "TWS", {{"vx", twist.linear.x}, {"wz", twist.angular.z}});
    return twist;
}

// ─────────────────────────────────────────────────────────────────────────────
std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance()
{
    pose_covariance_.fill(0.0);
    pose_covariance_[0]  = 0.1;   // x
    pose_covariance_[7]  = 0.1;   // y
    pose_covariance_[35] = 0.2;   // yaw
    return pose_covariance_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance()
{
    twist_covariance_.fill(0.0);
    twist_covariance_[0]  = 0.1;  // vx
    twist_covariance_[35] = 0.2;  // wz
    return twist_covariance_;
}

}
