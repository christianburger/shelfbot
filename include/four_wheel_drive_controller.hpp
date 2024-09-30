#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <vector>
#include <string>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace shelfbot
{

class FourWheelDriveController : public controller_interface::ControllerInterface
{
public:
  FourWheelDriveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_commands_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_sub_;
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmd_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  double wheel_separation_;
  double wheel_radius_;
  
  std::map<std::string, double> axis_positions_;
  std::map<std::string, double> axis_commands_;
  double encoder_resolution_;
};

}

#endif
