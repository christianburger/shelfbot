#ifndef FOUR_WHEEL_DRIVE_ODOMETRY_HPP
#define FOUR_WHEEL_DRIVE_ODOMETRY_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

namespace shelfbot {

class FourWheelDriveOdometry {
 public:
    FourWheelDriveOdometry(std::shared_ptr<rclcpp::Node> node, 
                          const rclcpp::Clock::SharedPtr& clock,
                          double wheel_separation, 
                          double wheel_radius);
    
    void update(const std::vector<double>& wheel_positions,
                const rclcpp::Duration& period);
    
    nav_msgs::msg::Odometry get_odometry() const;

 private:
    geometry_msgs::msg::Pose calculate_pose() const;
    geometry_msgs::msg::Twist calculate_twist(const std::vector<double>& wheel_positions, 
                                            const rclcpp::Duration& period);
    double calculate_linear_velocity(double left_wheel_vel, double right_wheel_vel);
    double calculate_angular_velocity(double left_wheel_vel, double right_wheel_vel);
    std::array<double, 36> calculate_pose_covariance();
    std::array<double, 36> calculate_twist_covariance();
    void broadcast_tf();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Clock::SharedPtr clock_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    double wheel_separation_;
    double wheel_radius_;

    double x_{0.0};
    double y_{0.0};

    double total_distance_{0.0};

    double theta_{0.0};
    double prev_left_front_pos_{0.0};
    double prev_right_front_pos_{0.0};
    double prev_left_rear_pos_{0.0};
    double prev_right_rear_pos_{0.0};
    double prev_linear_vel_{0.0};
    double prev_angular_vel_{0.0};

    std::array<double, 36> pose_covariance_;
    std::array<double, 36> twist_covariance_;
    std::vector<double> prev_wheel_positions_;
    bool initialized_{false};
};

}
#endif
