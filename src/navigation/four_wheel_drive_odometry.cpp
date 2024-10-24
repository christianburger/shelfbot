#include "four_wheel_drive_odometry.hpp"
#include "shelfbot_utils.hpp"

namespace shelfbot {

FourWheelDriveOdometry::FourWheelDriveOdometry(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const rclcpp::Clock::SharedPtr& clock, double wheel_separation, double wheel_radius) : node_(node), clock_(clock), wheel_separation_(wheel_separation), wheel_radius_(wheel_radius) {
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    log_info("FourWheelDriveOdometry", "Constructor", "Initialized with separation: " + std::to_string(wheel_separation_) + ", radius: " + std::to_string(wheel_radius_));
}

void FourWheelDriveOdometry::update(const std::vector<double>& wheel_positions, const rclcpp::Duration& period) {
    log_trace("FourWheelDriveOdometry", "update", "Processing wheel positions");
    
    double left_wheel_vel = ((wheel_positions[0] - prev_left_front_pos_) + 
                            (wheel_positions[2] - prev_left_rear_pos_)) / (2.0 * period.seconds());

    double right_wheel_vel = ((wheel_positions[1] - prev_right_front_pos_) + 
                             (wheel_positions[3] - prev_right_rear_pos_)) / (2.0 * period.seconds());

    double linear_vel = (left_wheel_vel + right_wheel_vel) * wheel_radius_ / 2.0;
    double angular_vel = (right_wheel_vel - left_wheel_vel) * wheel_radius_ / wheel_separation_;

    double dt = period.seconds();
    double delta_x = linear_vel * cos(theta_) * dt;
    double delta_y = linear_vel * sin(theta_) * dt;
    double delta_theta = angular_vel * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    prev_left_front_pos_ = wheel_positions[0];
    prev_right_front_pos_ = wheel_positions[1];
    prev_left_rear_pos_ = wheel_positions[2];
    prev_right_rear_pos_ = wheel_positions[3];

    publish_odometry(linear_vel, angular_vel);
}

void FourWheelDriveOdometry::publish_odometry(double linear_vel, double angular_vel) {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = clock_->now();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
    
    odom_msg->pose.pose.position.x = x_;
    odom_msg->pose.pose.position.y = y_;
    odom_msg->pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(theta_ / 2), std::cos(theta_ / 2)));
    
    odom_msg->twist.twist.linear.x = linear_vel;
    odom_msg->twist.twist.angular.z = angular_vel;

    odom_pub_->publish(std::move(odom_msg));
}

nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = clock_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(theta_ / 2), std::cos(theta_ / 2)));
    return odom;
}

}