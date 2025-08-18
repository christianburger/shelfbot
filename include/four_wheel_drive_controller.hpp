#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" // ADDED FOR DEBUGGING
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "four_wheel_drive_odometry.hpp"
#include "shelfbot_utils.hpp"

namespace shelfbot {

class FourWheelDriveController : public controller_interface::ControllerInterface {
 public:
  FourWheelDriveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Helper methods
  void publish_joint_states();
  void init_odometry();
  void update_odometry(const rclcpp::Duration& period);
  void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

  // ROS 2 Publishers and Subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr direct_cmd_subscriber_; // ADDED FOR DEBUGGING
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Controller state
  std::vector<std::string> joint_names_;
  std::vector<std::string> front_left_joint_names_;
  std::vector<std::string> back_left_joint_names_;
  std::vector<std::string> front_right_joint_names_;
  std::vector<std::string> back_right_joint_names_;
  std::map<std::string, double> axis_positions_;
  std::map<std::string, double> axis_commands_;
  
  // Odometry
  std::unique_ptr<FourWheelDriveOdometry> odometry_;
  
  // Robot parameters
  double wheel_separation_;
  double wheel_radius_;

  // Twist command state
  std::shared_ptr<geometry_msgs::msg::Twist> last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Duration cmd_vel_timeout_;

  // System clock is accessed via get_node()->get_clock()
};
} 

#endif