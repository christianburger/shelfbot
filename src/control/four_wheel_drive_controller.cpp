#include "four_wheel_drive_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace shelfbot
{

FourWheelDriveController::FourWheelDriveController()
  : hardware_interface::SystemInterface()
{
}

hardware_interface::CallbackReturn FourWheelDriveController::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
      return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
          RCLCPP_FATAL(
              rclcpp::get_logger("FourWheelDriveController"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", 
              joint.name.c_str(), joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2 ||
          joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
          RCLCPP_FATAL(
              rclcpp::get_logger("FourWheelDriveController"),
              "Joint '%s' has %zu state interface. 2 expected.", 
              joint.name.c_str(), joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
      }
  }

  joint_position_.resize(info_.joints.size(), 0);
  joint_velocity_.resize(info_.joints.size(), 0);
  joint_velocity_command_.resize(info_.joints.size(), 0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelDriveController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelDriveController::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FourWheelDriveController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FourWheelDriveController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      joint_position_[i] += joint_velocity_[i] * 0.01;  // Assume 10ms cycle time
      joint_velocity_[i] = joint_velocity_command_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      joint_velocity_[i] = joint_velocity_command_[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace shelfbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, hardware_interface::SystemInterface)
