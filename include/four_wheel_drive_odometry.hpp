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
    void publish_odometry(double linear_vel, double angular_vel);

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
};

}

#endif