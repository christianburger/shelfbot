#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <shelfbot_utils.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace shelfbot {

class FourWheelDriveHardwareInterface;

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
  std::shared_ptr<FourWheelDriveHardwareInterface> hardware_interface_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> axis_positions_;
  std::map<std::string, double> axis_commands_;

  double wheel_separation_;
  double wheel_radius_;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
  double prev_left_wheel_pos_ = 0.0;
  double prev_right_wheel_pos_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_state_pubs_;

  void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_joint_states();
  void calculate_odometry(const rclcpp::Duration& period);
};
}

#endif
