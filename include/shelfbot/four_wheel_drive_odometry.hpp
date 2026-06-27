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

class FourWheelDriveOdometry {
public:
  FourWheelDriveOdometry(
    std::shared_ptr<rclcpp::Node> node,
    const rclcpp::Clock::SharedPtr& clock,
    double wheel_separation,
    double wheel_radius,
    double gear_ratio);   // <-- added gear_ratio

  void update(
    const std::vector<double>& wheel_positions,
    const rclcpp::Duration& period);

  void broadcast_tf(const rclcpp::Time& stamp);
  nav_msgs::msg::Odometry get_odometry() const;

private:
  geometry_msgs::msg::Pose calculate_pose() const;
  geometry_msgs::msg::Twist calculate_twist(
    double left_diff_m, double right_diff_m, double dt_s);
  std::array<double, 36> calculate_pose_covariance();
  std::array<double, 36> calculate_twist_covariance();

  std::shared_ptr<rclcpp::Node>  node_;
  rclcpp::Clock::SharedPtr       clock_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>            tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     odom_pub_;

  double wheel_separation_;
  double wheel_radius_;
  double gear_ratio_;          // <-- new member variable
  bool   initialized_{false};

  double prev_left_pos_{0.0};
  double prev_right_pos_{0.0};

  double x_{0.0}, y_{0.0}, theta_{0.0};
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;
};

}
#endif