#include "four_wheel_drive_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <functional>

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using std::string;
using std::vector;
using std::to_string;
using std::placeholders::_1;

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController() : ControllerInterface() {
  ShelfbotUtils::log_info("FourWheelDriveController", "Constructor", "Called");
}

CallbackReturn FourWheelDriveController::on_init() {
  ShelfbotUtils::log_info("FourWheelDriveController", "on_init", "Called");
  try {
    auto_declare<vector<string>>("joints", vector<string>());
    ShelfbotUtils::log_info("FourWheelDriveController", "on_init", "Parameters declared");
  } catch (const std::exception& e) {
    ShelfbotUtils::log_error("FourWheelDriveController", "on_init", string("Exception: ") + e.what());
    return CallbackReturn::ERROR;
  }
  ShelfbotUtils::log_info("FourWheelDriveController", "on_init", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  ShelfbotUtils::log_info("FourWheelDriveController", "on_configure", "Called");
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    ShelfbotUtils::log_error("FourWheelDriveController", "on_configure", "No joints specified");
    return CallbackReturn::ERROR;
  }
  ShelfbotUtils::log_info("FourWheelDriveController", "on_configure", "Joints loaded: " + to_string(joint_names_.size()));

  for (const auto& joint : joint_names_) {
    axis_positions_[joint] = 0.0;
    axis_commands_[joint] = 0.0;
    ShelfbotUtils::log_info("FourWheelDriveController", "on_configure", "Initialized joint: " + joint);

    joint_state_pubs_[joint] = get_node()->create_publisher<std_msgs::msg::Float64>(joint + "/position", 10);
    ShelfbotUtils::log_info("FourWheelDriveController", "on_configure", "Created publisher for joint: " + joint);
  }

  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", 10, std::bind(&FourWheelDriveController::cmd_callback, this, _1));
  ShelfbotUtils::log_info("FourWheelDriveController", "on_configure", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  ShelfbotUtils::log_info("FourWheelDriveController", "on_activate", "Called");
  ShelfbotUtils::log_info("FourWheelDriveController", "on_activate", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  ShelfbotUtils::log_info("FourWheelDriveController", "on_deactivate", "Called");
  ShelfbotUtils::log_info("FourWheelDriveController", "on_deactivate", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  ShelfbotUtils::log_debug("FourWheelDriveController", "update", "Called");
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint_name = joint_names_[i];
    if (i < state_interfaces_.size() && i < command_interfaces_.size()) {
      axis_positions_[joint_name] = state_interfaces_[i].get_value();
      command_interfaces_[i].set_value(axis_commands_[joint_name]);
      ShelfbotUtils::log_debug("FourWheelDriveController", "update",
                "Joint " + joint_name + ": Position = " + to_string(axis_positions_[joint_name]) +
                ", Command = " + to_string(axis_commands_[joint_name]));
    }
  }
  publish_joint_states();
  return controller_interface::return_type::OK;
}

void FourWheelDriveController::cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  ShelfbotUtils::log_info("FourWheelDriveController", "cmd_callback", "Called");
  if (msg->data.size() != joint_names_.size()) {
    ShelfbotUtils::log_error("FourWheelDriveController", "cmd_callback", "Received command size does not match number of joints");
    return;
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_commands_[joint_names_[i]] = msg->data[i];
    ShelfbotUtils::log_info("FourWheelDriveController", "cmd_callback",
             "Setting " + joint_names_[i] + " command to " + to_string(msg->data[i]));
  }
}

void FourWheelDriveController::publish_joint_states() {
  for (const auto& joint : joint_names_) {
    std_msgs::msg::Float64 msg;
    msg.data = axis_positions_[joint];
    joint_state_pubs_[joint]->publish(msg);
    ShelfbotUtils::log_debug("FourWheelDriveController", "publish_joint_states",
              "Published position " + to_string(msg.data) + " for joint " + joint);
  }
}

InterfaceConfiguration FourWheelDriveController::command_interface_configuration() const {
  ShelfbotUtils::log_info("FourWheelDriveController", "command_interface_configuration", "Called");
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    ShelfbotUtils::log_info("FourWheelDriveController", "command_interface_configuration",
             "Added command interface: " + joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

InterfaceConfiguration FourWheelDriveController::state_interface_configuration() const {
  ShelfbotUtils::log_info("FourWheelDriveController", "state_interface_configuration", "Called");
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    ShelfbotUtils::log_info("FourWheelDriveController", "state_interface_configuration",
             "Added state interface: " + joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)
