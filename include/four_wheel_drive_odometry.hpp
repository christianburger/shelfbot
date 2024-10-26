#ifndef FOUR_WHEEL_DRIVE_ODOMETRY_HPP
#define FOUR_WHEEL_DRIVE_ODOMETRY_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace shelfbot {

class FourWheelDriveOdometry {
 public:
    FourWheelDriveOdometry(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, 
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

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    double wheel_separation_;
    double wheel_radius_;
    double x_{0.0};
    double y_{0.0};
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
};

}
#endif
