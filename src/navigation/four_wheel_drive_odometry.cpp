#include "four_wheel_drive_odometry.hpp"
#include "shelfbot_utils.hpp"

namespace shelfbot {

FourWheelDriveOdometry::FourWheelDriveOdometry(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, 
    const rclcpp::Clock::SharedPtr& clock,
    double wheel_separation, 
    double wheel_radius) 
    : node_(node), 
      clock_(clock), 
      wheel_separation_(wheel_separation), 
      wheel_radius_(wheel_radius),
      prev_wheel_positions_(4, 0.0) {
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pose_covariance_.fill(0.0);
    twist_covariance_.fill(0.0);
    log_info("FourWheelDriveOdometry", "Constructor", 
             "Initialized with separation: " + std::to_string(wheel_separation_) + 
             ", radius: " + std::to_string(wheel_radius_));
}

void FourWheelDriveOdometry::update(
    const std::vector<double>& wheel_positions, 
    const rclcpp::Duration& period) {
    
    if (wheel_positions == prev_wheel_positions_) {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = clock_->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";
        odom_msg->pose.pose = calculate_pose();
        odom_msg->twist.twist.linear.x = prev_linear_vel_;
        odom_msg->twist.twist.angular.z = prev_angular_vel_;
        odom_msg->pose.covariance = pose_covariance_;
        odom_msg->twist.covariance = twist_covariance_;
        odom_pub_->publish(std::move(odom_msg));
        return;
    }
    
    auto twist = calculate_twist(wheel_positions, period);
    
    double dt = period.seconds();
    double delta_x = twist.linear.x * cos(theta_) * dt;
    double delta_y = twist.linear.x * sin(theta_) * dt;
    double delta_theta = twist.angular.z * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    prev_left_front_pos_ = wheel_positions[0];
    prev_right_front_pos_ = wheel_positions[1];
    prev_left_rear_pos_ = wheel_positions[2];
    prev_right_rear_pos_ = wheel_positions[3];
    prev_wheel_positions_ = wheel_positions;

    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = clock_->now();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
    
    odom_msg->pose.pose = calculate_pose();
    odom_msg->twist.twist = twist;
    
    odom_msg->pose.covariance = calculate_pose_covariance();
    odom_msg->twist.covariance = calculate_twist_covariance();

    odom_pub_->publish(std::move(odom_msg));
    log_info("FourWheelDriveOdometry", "update", 
             "Published odometry - pos[" + std::to_string(x_) + "," + 
             std::to_string(y_) + "," + std::to_string(theta_) + "] " +
             "vel[" + std::to_string(twist.linear.x) + "," + 
             std::to_string(twist.angular.z) + "]");
}

geometry_msgs::msg::Pose FourWheelDriveOdometry::calculate_pose() const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(theta_ / 2), std::cos(theta_ / 2)));
    return pose;
}

geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist(
    const std::vector<double>& wheel_positions, 
    const rclcpp::Duration& period) {
    
    double dt = period.seconds();
    if (dt < 1e-9) {
        dt = 1e-9;
    }

    double left_wheel_vel = ((wheel_positions[0] - prev_left_front_pos_) + 
                            (wheel_positions[2] - prev_left_rear_pos_)) / (2.0 * dt);
    double right_wheel_vel = ((wheel_positions[1] - prev_right_front_pos_) + 
                             (wheel_positions[3] - prev_right_rear_pos_)) / (2.0 * dt);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = calculate_linear_velocity(left_wheel_vel, right_wheel_vel);
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = calculate_angular_velocity(left_wheel_vel, right_wheel_vel);

    return twist;
}

double FourWheelDriveOdometry::calculate_linear_velocity(
    double left_wheel_vel, 
    double right_wheel_vel) {
    prev_linear_vel_ = (left_wheel_vel + right_wheel_vel) * wheel_radius_ / 2.0;
    return prev_linear_vel_;
}

double FourWheelDriveOdometry::calculate_angular_velocity(
    double left_wheel_vel, 
    double right_wheel_vel) {
    prev_angular_vel_ = (right_wheel_vel - left_wheel_vel) * wheel_radius_ / wheel_separation_;
    return prev_angular_vel_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance() {
    pose_covariance_.fill(0.0);
    return pose_covariance_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance() {
    twist_covariance_.fill(0.0);
    return twist_covariance_;
}

nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = clock_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = calculate_pose();
    return odom;
}

}
