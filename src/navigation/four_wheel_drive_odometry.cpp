#include "shelfbot/four_wheel_drive_odometry.hpp"
#include "shelfbot/shelfbot_utils.hpp"

namespace shelfbot {

FourWheelDriveOdometry::FourWheelDriveOdometry(std::shared_ptr<rclcpp::Node> node, const rclcpp::Clock::SharedPtr& clock, double wheel_separation, double wheel_radius) : node_(node), clock_(clock), wheel_separation_(wheel_separation), wheel_radius_(wheel_radius), prev_wheel_positions_(4, 0.0) {

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    pose_covariance_.fill(0.0);
    twist_covariance_.fill(0.0);

    log_info("FourWheelDriveOdometry", "Constructor", "Initialized with separation: " + std::to_string(wheel_separation_) + ", radius: " + std::to_string(wheel_radius_));
}

void FourWheelDriveOdometry::update(const std::vector<double>& wheel_positions, const rclcpp::Duration& period) {
    // On the first run, initialize prev_wheel_positions_ and exit
    if (!initialized_)
    {
        prev_wheel_positions_ = wheel_positions;
        initialized_ = true;
        return;
    }

    // --- Standard Skid-Steer Odometry Calculation ---

    // Average the positions of the wheels on each side.
    // Note: The specific indices depend on the joint order from the URDF.
    // Assuming [0, 2] are left wheels and [1, 3] are right wheels.
    double left_pos = (wheel_positions[0] + wheel_positions[2]) / 2.0;
    double right_pos = (wheel_positions[1] + wheel_positions[3]) / 2.0;

    // Calculate the change in position for each side since the last update
    double prev_left_pos = (prev_wheel_positions_[0] + prev_wheel_positions_[2]) / 2.0;
    double prev_right_pos = (prev_wheel_positions_[1] + prev_wheel_positions_[3]) / 2.0;
    double left_diff = left_pos - prev_left_pos;
    double right_diff = right_pos - prev_right_pos;

    // Convert the change in wheel angle (radians) to linear distance (meters)
    double left_linear = left_diff * wheel_radius_;
    double right_linear = right_diff * wheel_radius_;

    // Calculate the robot's forward distance and rotation using standard differential drive kinematics
    const double forward_distance = (left_linear + right_linear) / 2.0;
    const double rotation = (right_linear - left_linear) / wheel_separation_;

    // Integrate the motion to update the robot's pose
    theta_ += rotation;
    x_ += forward_distance * cos(theta_);
    y_ += forward_distance * sin(theta_);

    // Update previous wheel positions for the next cycle
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = clock_->now();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_footprint";
    odom_msg->pose.pose = calculate_pose();
    odom_msg->pose.covariance = calculate_pose_covariance();
    odom_msg->twist.twist = calculate_twist(wheel_positions, period);
    odom_msg->twist.covariance = calculate_twist_covariance();

    prev_wheel_positions_ = wheel_positions;

    // Logging for debug
    std::stringstream ss;
    ss << "Received: [" << wheel_positions[0] << ", " << wheel_positions[1] << ", " << wheel_positions[2] << ", " << wheel_positions[3] << "] | "
       << "Calculated: [L:" << left_pos << " R:" << right_pos << " | dL:" << left_diff << " dR:" << right_diff << "] | "
       << "Published: [X:" << x_ << " Y:" << y_ << " T:" << theta_ << "]";
    log_info("FourWheelDriveOdometry", "Update", ss.str());

    odom_pub_->publish(std::move(odom_msg));
    broadcast_tf();
}

geometry_msgs::msg::Pose FourWheelDriveOdometry::calculate_pose() const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.position.z = 0.0;
    double final_theta = theta_;
    pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(final_theta / 2), std::cos(final_theta / 2)));
    
    log_info("FourWheelDriveOdometry", "CalculatePose",
             "X: " + std::to_string(x_) +
             " Y: " + std::to_string(y_) +
             " Theta: " + std::to_string(theta_) +
             " Final Theta: " + std::to_string(final_theta));
             
    return pose;
}

geometry_msgs::msg::Twist FourWheelDriveOdometry::calculate_twist( const std::vector<double>& wheel_positions, const rclcpp::Duration& period) {
    
    double dt = period.seconds();
    if (dt < 1e-9) {
        dt = 1e-9;
    }

    // --- Standard Skid-Steer Twist Calculation ---
    double prev_left_pos = (prev_wheel_positions_[0] + prev_wheel_positions_[2]) / 2.0;
    double prev_right_pos = (prev_wheel_positions_[1] + prev_wheel_positions_[3]) / 2.0;

    double left_pos = (wheel_positions[0] + wheel_positions[2]) / 2.0;
    double right_pos = (wheel_positions[1] + wheel_positions[3]) / 2.0;

    double left_wheel_vel = (left_pos - prev_left_pos) / dt;
    double right_wheel_vel = (right_pos - prev_right_pos) / dt;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = calculate_linear_velocity(left_wheel_vel, right_wheel_vel);
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = calculate_angular_velocity(left_wheel_vel, right_wheel_vel);

    return twist;
}

double FourWheelDriveOdometry::calculate_linear_velocity( double left_wheel_vel, double right_wheel_vel) {
    double left_linear = left_wheel_vel * wheel_radius_;
    double right_linear = right_wheel_vel * wheel_radius_;
    prev_linear_vel_ = (left_linear + right_linear) / 2.0;
    return prev_linear_vel_;
}

double FourWheelDriveOdometry::calculate_angular_velocity( double left_wheel_vel, double right_wheel_vel) {
    double left_linear = left_wheel_vel * wheel_radius_;
    double right_linear = right_wheel_vel * wheel_radius_;
    prev_angular_vel_ = (right_linear - left_linear) / wheel_separation_;
    return prev_angular_vel_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_pose_covariance() {
    pose_covariance_.fill(0.0);
    pose_covariance_[0] = 0.1;   // x
    pose_covariance_[7] = 0.1;   // y
    pose_covariance_[35] = 0.2;  // yaw
    return pose_covariance_;
}

std::array<double, 36> FourWheelDriveOdometry::calculate_twist_covariance() {
    twist_covariance_.fill(0.0);
    twist_covariance_[0] = 0.1;   // linear x
    twist_covariance_[35] = 0.2;  // angular z
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

void FourWheelDriveOdometry::broadcast_tf() {
    // First transform: odom -> base_footprint
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = clock_->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    double final_theta = theta_ - 1.57079632679;
    odom_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(final_theta / 2), std::cos(final_theta / 2)));

    tf_broadcaster_->sendTransform(odom_tf);
}

}
