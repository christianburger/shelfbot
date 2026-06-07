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

    // ── LEFT/RIGHT SIDE AVERAGING ─────────────────────────────────────────────
    //
    // This is a differential-drive (skid-steer) odometry model. The robot has
    // four wheels but only two independent velocity degrees of freedom: left side
    // and right side. We reduce each side to a single representative position by
    // averaging the front and rear wheels on that side.
    //
    // JOINT INDEX MAP (matches YAML joint_names / hw_positions_ order set in
    // FourWheelDriveHardwareInterface::export_state_interfaces()):
    //   [0] front_left   — already sign-corrected by read() to forward-positive
    //   [1] front_right  — forward-positive as received from firmware
    //   [2] back_left    — already sign-corrected by read() to forward-positive
    //   [3] back_right   — forward-positive as received from firmware
    //
    // WHY AVERAGE LEFT+RIGHT FRONTS vs LEFT+RIGHT REARS WOULD BE WRONG:
    // The previous (buggy) implementation treated [0]+[1] as one axis and
    // [2]+[3] as the other axis, meaning it was computing "front axle average"
    // and "rear axle average". For a skid-steer robot driving straight, those
    // two quantities are equal — the difference (which drives the rotation
    // estimate) is always ~0, while the sum (which drives the translation
    // estimate) doubled the actual displacement. This caused Nav2 to see near-
    // zero forward progress and infinite spin, making the robot spin in place.
    //
    // CORRECT MODEL:
    //   left_pos  = average of FL and BL  (both forward-positive after read() flip)
    //   right_pos = average of FR and BR
    //   fwd_dist  = (left_diff + right_diff) / 2   ← translation
    //   rotation  = (right_diff - left_diff) / wheel_separation_  ← yaw
    //
    // !! DO NOT CHANGE THESE INDEX GROUPINGS !!
    // Swapping to [0]+[1] and [2]+[3] restores the original bug.
    // ─────────────────────────────────────────────────────────────────────────
    double left_pos  = (wheel_positions[0] + wheel_positions[2]) * 0.5;   // FL + BL
    double right_pos = (wheel_positions[1] + wheel_positions[3]) * 0.5;   // FR + BR

    if (!initialized_) {
        prev_left_pos_  = left_pos;
        prev_right_pos_ = right_pos;
        initialized_ = true;
        log_zip("ODO", "LATCH", {
            {"left", left_pos}, {"right", right_pos}
        });
        log_info("FourWheelDriveOdometry", "update",
                 "Odometry initialized with left/right wheel positions");
        return;
    }

    double left_diff  = (left_pos  - prev_left_pos_)  * wheel_radius_;
    double right_diff = (right_pos - prev_right_pos_) * wheel_radius_;

    // ── UPDATE prev_* IMMEDIATELY AFTER COMPUTING THE DIFFS ──────────────────
    // prev_left_pos_ and prev_right_pos_ are updated here, right after the diffs
    // are computed and before anything else. calculate_twist() no longer reads
    // these fields at all — it receives left_diff and right_diff directly — so
    // there is no longer any call-order constraint between prev_* assignment and
    // the rest of update(). This replaces the previous arrangement where prev_*
    // had to stay at the very end of the function to avoid corrupting twist.
    // ─────────────────────────────────────────────────────────────────────────
    prev_left_pos_  = left_pos;
    prev_right_pos_ = right_pos;

    // Forward displacement = average of left and right arc lengths
    double fwd_dist = (left_diff + right_diff) * 0.5;
    // Rotational displacement = (right - left) / wheel_separation (positive = CCW / turning left)
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

  // ── TF YAW OFFSET ────────────────────────────────────────────────────────
  // The TF broadcast applies a fixed −90° (−π/2) offset to theta_ before
  // converting to a quaternion. This corrects a fixed orientation error between
  // the robot's physical "forward" direction and the coordinate frame reported
  // by the firmware's encoder convention.
  //
  // The offset is applied HERE in broadcast_tf but NOT in calculate_pose().
  // This is intentional: the nav_msgs/Odometry pose field (used by Nav2's EKF
  // and costmap) uses the uncorrected theta_ so that sensor fusion remains
  // consistent. The TF tree (used for visualization and frame lookups) uses the
  // corrected angle so that the robot model appears correctly oriented in RViz.
  //
  // If the robot appears rotated 90° sideways in RViz: this offset is why and
  // it is correct. Do not remove it.
  // ─────────────────────────────────────────────────────────────────────────
  double final_theta = theta_ - 1.57079632679;  // theta_ - π/2
  odom_tf.transform.rotation =
      tf2::toMsg(tf2::Quaternion(
        0, 0,
        std::sin(final_theta / 2.0),
        std::cos(final_theta / 2.0)));

  tf_broadcaster_->sendTransform(odom_tf);

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

  // ── NO YAW OFFSET HERE ───────────────────────────────────────────────────
  // The odom message pose intentionally uses theta_ WITHOUT the −π/2 correction
  // applied in broadcast_tf(). Nav2's EKF and the costmap consume this pose
  // for sensor fusion; it must be in the robot's internal odometry frame and
  // must be consistent with the twist field. Applying the offset here would
  // desynchronise position integration from the twist, corrupting EKF estimates.
  // See broadcast_tf() for the explanation of why the TF branch applies the
  // correction while this branch does not.
  // ─────────────────────────────────────────────────────────────────────────
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
    double left_diff_m, double right_diff_m, double dt_s)
{
  // left_diff_m and right_diff_m are arc-length displacements in metres
  // already computed by update() for this cycle: (pos_now - pos_prev) * wheel_radius_.
  // dt_s is the cycle period in seconds (already validated > 0 by update()).
  //
  // This function deliberately does NOT read prev_left_pos_ / prev_right_pos_.
  // Receiving the pre-computed values means the function is call-order-independent
  // with respect to when prev_* are written in update().
  if (dt_s < 1e-9) { dt_s = 1e-9; }

  double left_vel  = left_diff_m  / dt_s;
  double right_vel = right_diff_m / dt_s;

  geometry_msgs::msg::Twist twist;
  twist.linear.x  = (left_vel + right_vel) * 0.5;
  twist.angular.z = (right_vel - left_vel) / wheel_separation_;

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
