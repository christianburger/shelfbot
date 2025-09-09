// four_wheel_drive_odometry.cpp

#include "shelfbot/four_wheel_drive_odometry.hpp"
#include "shelfbot/shelfbot_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace shelfbot {

// Constructor matches header:
//   FourWheelDriveOdometry(std::shared_ptr<rclcpp::Node>, const rclcpp::Clock::SharedPtr&, double, double)
FourWheelDriveOdometry::FourWheelDriveOdometry(
    std::shared_ptr<rclcpp::Node> node,
    const rclcpp::Clock::SharedPtr& clock,
    double wheel_separation,
    double wheel_radius)
  : node_(node),
    clock_(clock),
    wheel_separation_(wheel_separation),
    wheel_radius_(wheel_radius),
    prev_wheel_positions_(4, 0.0)
{
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  pose_covariance_.fill(0.0);
  twist_covariance_.fill(0.0);

  log_info(
    "FourWheelDriveOdometry",
    "Constructor",
    "Initialized with separation: " + std::to_string(wheel_separation_) +
    ", radius: " + std::to_string(wheel_radius_));
}

// update(const std::vector<double>&, const rclcpp::Duration&)
void FourWheelDriveOdometry::update(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period)
{
  if (!initialized_)
  {
    prev_wheel_positions_ = wheel_positions;
    initialized_ = true;
    return;
  }

  // 1) Integrate skid-steer odometry
  double front_pos = (wheel_positions[0] + wheel_positions[1]) * 0.5;
  double back_pos  = (wheel_positions[2] + wheel_positions[3]) * 0.5;

  double prev_front = (prev_wheel_positions_[0] + prev_wheel_positions_[1]) * 0.5;
  double prev_back  = (prev_wheel_positions_[2] + prev_wheel_positions_[3]) * 0.5;

  double front_diff = front_pos - prev_front;
  double back_diff  = back_pos  - prev_back;

  double front_linear = front_diff * wheel_radius_;
  double back_linear  = back_diff  * wheel_radius_;

  double forward_distance = (front_linear - back_linear) * 0.5;
  double rotation         = (front_linear + back_linear) * 0.5;

  theta_ += rotation;
  x_     += forward_distance * std::cos(theta_);
  y_     += forward_distance * std::sin(theta_);

  prev_wheel_positions_ = wheel_positions;

  // 2) Single timestamp for odom and TF
  auto stamp = clock_->now();

  // Build Odometry message
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->header.stamp    = stamp;
  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id  = "base_footprint";

  odom_msg->pose.pose       = calculate_pose();
  odom_msg->pose.covariance = calculate_pose_covariance();

  odom_msg->twist.twist      = calculate_twist(wheel_positions, period);
  odom_msg->twist.covariance = calculate_twist_covariance();

  // 3) Debug logging (unchanged)
  log_info("FourWheelDriveOdometry", "Update", "[odometry_calculation] --- Odometry Update ---");
  log_info("FourWheelDriveOdometry", "Update",
           "\tRaw Wheel Positions:  [" +
           std::to_string(wheel_positions[0]) + ", " +
           std::to_string(wheel_positions[1]) + ", " +
           std::to_string(wheel_positions[2]) + ", " +
           std::to_string(wheel_positions[3]) + "]");
  log_info("FourWheelDriveOdometry", "Update",
           "\tPrev Wheel Positions: [" +
           std::to_string(prev_wheel_positions_[0]) + ", " +
           std::to_string(prev_wheel_positions_[1]) + ", " +
           std::to_string(prev_wheel_positions_[2]) + ", " +
           std::to_string(prev_wheel_positions_[3]) + "]");
  log_info("FourWheelDriveOdometry", "Update",
           "\tAvg Positions: front=" + std::to_string(front_pos) +
           ", back=" + std::to_string(back_pos));
  log_info("FourWheelDriveOdometry", "Update",
           "\tPosition Diffs: front=" + std::to_string(front_diff) +
           ", back=" + std::to_string(back_diff));
  log_info("FourWheelDriveOdometry", "Update",
           "\tLinear Diffs: front=" + std::to_string(front_linear) +
           ", back=" + std::to_string(back_linear));
  log_info("FourWheelDriveOdometry", "Update",
           "\tCalculated Deltas: forward_distance=" + std::to_string(forward_distance) +
           ", rotation=" + std::to_string(rotation));
  log_info("FourWheelDriveOdometry", "Update",
           "\tFinal Pose: x=" + std::to_string(x_) +
           ", y=" + std::to_string(y_) +
           ", theta=" + std::to_string(theta_));

  odom_pub_->publish(std::move(odom_msg));

  // 4) Broadcast TF with same stamp
  broadcast_tf(stamp);
}

// broadcast_tf(const rclcpp::Time&)
void FourWheelDriveOdometry::broadcast_tf(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp    = stamp;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id  = "base_footprint";

  odom_tf.transform.translation.x = x_;
  odom_tf.transform.translation.y = y_;
  odom_tf.transform.translation.z = 0.0;

  double final_theta = theta_ - 1.57079632679;  // adjust if needed
  odom_tf.transform.rotation =
      tf2::toMsg(tf2::Quaternion(
        0, 0,
        std::sin(final_theta / 2.0),
        std::cos(final_theta / 2.0)));

  tf_broadcaster_->sendTransform(odom_tf);
}

// get_odometry() const
nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = clock_->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_footprint";
  odom.pose.pose       = calculate_pose();
  return odom;
}

// calculate_pose() const
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

  log_info(
    "FourWheelDriveOdometry",
    "CalculatePose",
    "X: " + std::to_string(x_) +
    " Y: " + std::to_string(y_) +
    " Theta: " + std::to_string(theta_) +
    " Final Theta: " + std::to_string(final_theta));

  return pose;
}

// calculate_twist(...)
geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period)
{
  double dt = period.seconds();
  if (dt < 1e-9) { dt = 1e-9; }

  double prev_front = (prev_wheel_positions_[0] + prev_wheel_positions_[1]) * 0.5;
  double prev_back  = (prev_wheel_positions_[2] + prev_wheel_positions_[3]) * 0.5;

  double front_pos = (wheel_positions[0] + wheel_positions[1]) * 0.5;
  double back_pos  = (wheel_positions[2] + wheel_positions[3]) * 0.5;

  double front_vel = (front_pos - prev_front) / dt;
  double back_vel  = (back_pos  - prev_back) / dt;

  geometry_msgs::msg::Twist twist;
  twist.linear.x  = (front_vel - back_vel) * wheel_radius_ * 0.5;
  twist.linear.y  = 0.0;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = (front_vel + back_vel) * wheel_radius_ * 0.5;
  return twist;
}

// calculate_pose_covariance()
std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance()
{
  pose_covariance_.fill(0.0);
  pose_covariance_[0]  = 0.1;
  pose_covariance_[7]  = 0.1;
  pose_covariance_[35] = 0.2;
  return pose_covariance_;
}

// calculate_twist_covariance()
std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance()
{
  twist_covariance_.fill(0.0);
  twist_covariance_[0]  = 0.1;
  twist_covariance_[35] = 0.2;
  return twist_covariance_;
}

}  // namespace shelfbot
