#ifndef SHELFBOT_FOUR_WHEEL_DRIVE_ODOMETRY_HPP_
#define SHELFBOT_FOUR_WHEEL_DRIVE_ODOMETRY_HPP_

#include <memory>
#include <vector>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace shelfbot {

// ── FourWheelDriveOdometry ────────────────────────────────────────────────────
//
// Differential-drive (skid-steer) odometry for a four-wheeled robot.
//
// SIGN CONVENTION (all fields after the read()-side correction in the HW interface):
//   Positive wheel position / velocity = wheel rotating in the forward direction.
//   This is true for ALL four wheels after FourWheelDriveHardwareInterface::read()
//   negates the left-side encoder readings.
//
// COORDINATE FRAME:
//   Pose (x_, y_, theta_) is accumulated in the odom frame.
//   theta_ = 0 means the robot faces the +X direction of the odom frame.
//   theta_ increases CCW (positive wz = left turn), consistent with REP-103.
//
// TWO SEPARATE PREVIOUS-POSITION FIELDS:
//   prev_left_pos_ and prev_right_pos_ track the left-side average and right-side
//   average respectively. This replaced the old prev_wheel_positions_ vector which
//   used the wrong front/back grouping. See update() for the detailed explanation.
// ─────────────────────────────────────────────────────────────────────────────

class FourWheelDriveOdometry
{
public:
  FourWheelDriveOdometry(
    std::shared_ptr<rclcpp::Node> node,
    const rclcpp::Clock::SharedPtr& clock,
    double wheel_separation,
    double wheel_radius);

  // Called every control cycle from the hardware interface's read() path.
  // wheel_positions must already have the left-side sign correction applied
  // (i.e., forward motion causes all four values to increase).
  void update(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period);

  // Broadcasts the odom → base_footprint TF with the −π/2 yaw correction.
  // Called internally by update(); exposed for testing.
  void broadcast_tf(const rclcpp::Time & stamp);

  nav_msgs::msg::Odometry get_odometry() const;

private:
  // Returns pose WITHOUT the −π/2 TF correction (for nav_msgs/Odometry).
  geometry_msgs::msg::Pose calculate_pose() const;

  // Returns instantaneous body-frame twist from the arc-length deltas that
  // update() already computed this cycle.
  //
  // Parameters are arc-length displacements in metres (not raw encoder counts,
  // not radians): left_diff = (left_pos_now - left_pos_prev) * wheel_radius_,
  // and the same for right. dt is the period in seconds.
  //
  // Receiving the pre-computed diffs instead of re-deriving them from prev_*
  // makes this function call-order-independent: prev_left_pos_ / prev_right_pos_
  // may be updated at any point in update() without affecting the result here.
  geometry_msgs::msg::Twist calculate_twist(
    double left_diff_m, double right_diff_m, double dt_s);

  std::array<double, 36> calculate_pose_covariance();
  std::array<double, 36> calculate_twist_covariance();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double wheel_separation_;
  double wheel_radius_;
  bool initialized_{false};

  // ── Per-side previous positions ───────────────────────────────────────────
  // Stored as the averaged left-side and right-side positions, not the raw
  // per-wheel positions. This enforces the correct skid-steer grouping:
  //   left  = average of wheel_positions[0] (FL) and wheel_positions[2] (BL)
  //   right = average of wheel_positions[1] (FR) and wheel_positions[3] (BR)
  //
  // !! DO NOT replace these with a single prev_wheel_positions_ vector !!
  // Using a single vector makes it tempting to index individual wheels and
  // reintroduces the front/back grouping bug that was the original failure mode.
  // ─────────────────────────────────────────────────────────────────────────
  double prev_left_pos_{0.0};
  double prev_right_pos_{0.0};

  double x_{0.0}, y_{0.0}, theta_{0.0};
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;
};

}  // namespace shelfbot

#endif  // SHELFBOT_FOUR_WHEEL_DRIVE_ODOMETRY_HPP_
