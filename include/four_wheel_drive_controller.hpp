#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <vector>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  double wheel_separation_;
  double wheel_radius_;
};

}

#endif