#include "four_wheel_drive_odometry.hpp"
#include "shelfbot_utils.hpp"

namespace shelfbot {

FourWheelDriveOdometry::FourWheelDriveOdometry(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, 
    const rclcpp::Clock::SharedPtr& clock,
    double wheel_separation, 
    double wheel_radius) : node_(node), clock_(clock), wheel_separation_(wheel_separation), wheel_radius_(wheel_radius) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        log_info("FourWheelDriveOdometry", "Constructor", "Initialized with separation: " + std::to_string(wheel_separation_) + ", radius: " + std::to_string(wheel_radius_));
    }

    void FourWheelDriveOdometry::update(const std::vector<double>& wheel_positions, const rclcpp::Duration& period) {
        log_debug("FourWheelDriveOdometry", "update", "Processing wheel positions: [" + 
              std::to_string(wheel_positions[0]) + ", " + 
              std::to_string(wheel_positions[1]) + ", " + 
              std::to_string(wheel_positions[2]) + ", " + 
              std::to_string(wheel_positions[3]) + "]");
    
        auto twist = calculate_twist(wheel_positions, period);
        log_trace("FourWheelDriveOdometry", "update", "Calculated twist - linear.x: " + 
              std::to_string(twist.linear.x) + ", angular.z: " + 
              std::to_string(twist.angular.z));
    
        double dt = period.seconds();
        double delta_x = twist.linear.x * cos(theta_) * dt;
        double delta_y = twist.linear.x * sin(theta_) * dt;
        double delta_theta = twist.angular.z * dt;

        log_debug("FourWheelDriveOdometry", "update", "Delta values - x: " + 
              std::to_string(delta_x) + ", y: " + 
              std::to_string(delta_y) + ", theta: " + 
              std::to_string(delta_theta));

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        log_trace("FourWheelDriveOdometry", "update", "Updated pose - x: " + 
              std::to_string(x_) + ", y: " + 
              std::to_string(y_) + ", theta: " + 
              std::to_string(theta_));

        prev_left_front_pos_ = wheel_positions[0];
        prev_right_front_pos_ = wheel_positions[1];
        prev_left_rear_pos_ = wheel_positions[2];
        prev_right_rear_pos_ = wheel_positions[3];

        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = clock_->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";
    
        odom_msg->pose.pose = calculate_pose();
        odom_msg->twist.twist = twist;
    
        odom_msg->pose.covariance = calculate_pose_covariance();
        odom_msg->twist.covariance = calculate_twist_covariance();

        odom_pub_->publish(std::move(odom_msg));
        log_info("FourWheelDriveOdometry", "update", "Published odometry message");
    }

geometry_msgs::msg::Pose FourWheelDriveOdometry::calculate_pose() const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0.0;
    pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(theta_ / 2), std::cos(theta_ / 2)));
    
    log_trace("FourWheelDriveOdometry", "calculate_pose", "Calculated pose - x: " + 
              std::to_string(pose.position.x) + ", y: " + 
              std::to_string(pose.position.y) + ", theta: " + 
              std::to_string(theta_));
    return pose;
}

geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist(
    const std::vector<double>& wheel_positions, 
    const rclcpp::Duration& period) {
    
    double left_wheel_vel = ((wheel_positions[0] - prev_left_front_pos_) + 
                            (wheel_positions[2] - prev_left_rear_pos_)) / (2.0 * period.seconds());
    double right_wheel_vel = ((wheel_positions[1] - prev_right_front_pos_) + 
                             (wheel_positions[3] - prev_right_rear_pos_)) / (2.0 * period.seconds());

    log_debug("FourWheelDriveOdometry", "calculate_twist", "Wheel velocities - left: " + 
              std::to_string(left_wheel_vel) + ", right: " + 
              std::to_string(right_wheel_vel));

    geometry_msgs::msg::Twist twist;
    twist.linear.x = calculate_linear_velocity(left_wheel_vel, right_wheel_vel);
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = calculate_angular_velocity(left_wheel_vel, right_wheel_vel);

    log_trace("FourWheelDriveOdometry", "calculate_twist", "Calculated velocities - linear: " + 
              std::to_string(twist.linear.x) + ", angular: " + 
              std::to_string(twist.angular.z));
    return twist;
}

double FourWheelDriveOdometry::calculate_linear_velocity(
    double left_wheel_vel, 
    double right_wheel_vel) {
    double velocity = (left_wheel_vel + right_wheel_vel) * wheel_radius_ / 2.0;
    log_trace("FourWheelDriveOdometry", "calculate_linear_velocity", 
              "Linear velocity: " + std::to_string(velocity));
    return velocity;
}

double FourWheelDriveOdometry::calculate_angular_velocity(
    double left_wheel_vel, 
    double right_wheel_vel) {
    double velocity = (right_wheel_vel - left_wheel_vel) * wheel_radius_ / wheel_separation_;
    log_trace("FourWheelDriveOdometry", "calculate_angular_velocity", 
              "Angular velocity: " + std::to_string(velocity));
    return velocity;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance() {
    std::array<double, 36> covariance;
    covariance.fill(0.0);
    log_trace("FourWheelDriveOdometry", "calculate_pose_covariance", "Covariance matrix calculated");
    return covariance;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance() {
    std::array<double, 36> covariance;
    covariance.fill(0.0);
    log_trace("FourWheelDriveOdometry", "calculate_twist_covariance", "Covariance matrix calculated");
    return covariance;
}

nav_msgs::msg::Odometry FourWheelDriveOdometry::get_odometry() const {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = clock_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = calculate_pose();
    log_trace("FourWheelDriveOdometry", "get_odometry", "Retrieved current odometry state");
    return odom;
}

}
