#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace shelfbot
{

class FourWheelDriveController : public hardware_interface::SystemInterface
{
public:
  FourWheelDriveController();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_velocity_command_;
};

}  // namespace shelfbot

#endif  // FOUR_WHEEL_DRIVE_CONTROLLER_HPP
