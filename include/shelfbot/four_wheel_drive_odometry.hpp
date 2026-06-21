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
// PUBLISHER TOPIC:
//   /wheel_odom_raw — raw encoder-derived odometry consumed by the EKF.
//   When the EKF (robot_localization) is active in the launch file, the EKF
//   re-publishes filtered odometry on /odom (remapped from odometry/filtered).
//   If no EKF is present, add a remapping in ros2_control_node to remap
//   /wheel_odom_raw → /odom, and set odometry_publish_tf=true.
//
// TF BROADCAST:
//   Controlled by the odometry_publish_tf ROS parameter (default: true).
//   Set to false when the EKF is running — the EKF owns odom→base_footprint.
//   Two nodes broadcasting the same TF edge causes "extrapolation into the
//   future" errors in the controller_server.
//
// SIGN CONVENTION (after read()-side correction in the HW interface):
//   Positive wheel position/velocity = forward direction for ALL four wheels.
//
// COORDINATE FRAME:
//   theta_ = 0 means the robot faces +X in the odom frame.
//   theta_ increases CCW (positive wz = left turn), consistent with REP-103.
//
// YAW OFFSET:
//   broadcast_tf() applies −π/2 to theta_ before publishing the TF.
//   calculate_pose() does NOT apply this offset (keeps /odom consistent
//   with the twist field for EKF sensor fusion).
// ─────────────────────────────────────────────────────────────────────────────

class FourWheelDriveOdometry {

public:
  FourWheelDriveOdometry(
      std::shared_ptr<rclcpp::Node> node,
      const rclcpp::Clock::SharedPtr& clock,
      double wheel_separation,
      double wheel_radius,
      bool publish_tf = false);   // default false: EKF owns the TF

  // Called every control cycle from the hardware interface's read() path.
  void update(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period);

  // Broadcasts the odom→base_footprint TF with the −π/2 yaw correction.
  // Only called when publish_tf_=true (no EKF in the stack).
  void broadcast_tf(const rclcpp::Time& stamp);

  nav_msgs::msg::Odometry get_odometry() const;

private:
  geometry_msgs::msg::Pose calculate_pose() const;

  geometry_msgs::msg::Twist calculate_twist(
    double left_diff_m, double right_diff_m, double dt_s);

  std::array<double, 36> calculate_pose_covariance();
  std::array<double, 36> calculate_twist_covariance();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Clock::SharedPtr clock_;

  // Optional — only created when publish_tf_=true.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double wheel_separation_;
  double wheel_radius_;
  bool initialized_{false};

  // When true: this node broadcasts odom→base_footprint TF (no EKF).
  // When false: EKF owns the TF; tf_broadcaster_ is not created.
  bool publish_tf_{true};

  // Per-side previous positions (averaged FL+BL and FR+BR).
  // DO NOT replace with a single prev_wheel_positions_ vector — that
  // reintroduces the front/back grouping bug.
  double prev_left_pos_{0.0};
  double prev_right_pos_{0.0};

  double x_{0.0}, y_{0.0}, theta_{0.0};
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};
};

}

#endif  // SHELFBOT_FOUR_WHEEL_DRIVE_ODOMETRY_HPP_
