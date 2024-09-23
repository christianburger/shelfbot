#include "four_wheel_drive_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>

namespace shelfbot
{

FourWheelDriveController::FourWheelDriveController()
  : controller_interface::ControllerInterface()
{
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: FourWheelDriveController constructor called");
}

controller_interface::CallbackReturn FourWheelDriveController::on_init()
{
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: Initializing FourWheelDriveController");
    
    try {
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"), "Exception thrown during init stage with message: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: FourWheelDriveController initialized");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: Configuring FourWheelDriveController");
    
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"), "No joints specified");
        return controller_interface::CallbackReturn::ERROR;
    }

    joint_positions_.resize(joint_names_.size(), 0.0);
    joint_velocities_.resize(joint_names_.size(), 0.0);
    joint_commands_.resize(joint_names_.size(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: FourWheelDriveController configured");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: Activating FourWheelDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: Deactivating FourWheelDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    RCLCPP_DEBUG(rclcpp::get_logger("FourWheelDriveController"), "TROUBLESHOOTING: Updating FourWheelDriveController");
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        joint_positions_[i] = state_interfaces_[i].get_value();
        joint_velocities_[i] = state_interfaces_[i + joint_names_.size()].get_value();
        command_interfaces_[i].set_value(joint_commands_[i]);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration FourWheelDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

controller_interface::InterfaceConfiguration FourWheelDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

}  // namespace shelfbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)
