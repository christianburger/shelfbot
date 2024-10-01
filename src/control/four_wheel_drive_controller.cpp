#include "four_wheel_drive_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController()
              : controller_interface::ControllerInterface() {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: Constructor called");
}

controller_interface::CallbackReturn FourWheelDriveController::on_init() {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_init called");
            try {
              auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
              RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                          "TROUBLESHOOTING: Parameters declared");
            } catch (const std::exception& e) {
              RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"),
                           "TROUBLESHOOTING: Exception in on_init: %s", e.what());
              return controller_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_init completed successfully");
            return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_configure(
              const rclcpp_lifecycle::State& previous_state) {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_configure called");
            joint_names_ = get_node()->get_parameter("joints").as_string_array();
            if (joint_names_.empty()) {
              RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"),
                           "TROUBLESHOOTING: No joints specified");
              return controller_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: Joints loaded: %zu", joint_names_.size());

            for (const auto& joint : joint_names_) {
              axis_positions_[joint] = 0.0;
              axis_commands_[joint] = 0.0;
              RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                          "TROUBLESHOOTING: Initialized joint: %s", joint.c_str());

              joint_state_pubs_[joint] =
                  get_node()->create_publisher<std_msgs::msg::Float64>(
                      joint + "/position", 10);
              RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                          "TROUBLESHOOTING: Created publisher for joint: %s",
                          joint.c_str());
            }

            cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
                "~/commands", 10,
                std::bind(&FourWheelDriveController::cmd_callback, this,
                          std::placeholders::_1));
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: Command subscription created");

            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_configure completed successfully");
            return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_activate(
              const rclcpp_lifecycle::State& previous_state) {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_activate called");
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_activate completed successfully");
            return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_deactivate(
              const rclcpp_lifecycle::State& previous_state) {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_deactivate called");
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: on_deactivate completed successfully");
            return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(
              const rclcpp::Time& time, const rclcpp::Duration& period) {
            RCLCPP_DEBUG(rclcpp::get_logger("FourWheelDriveController"),
                         "TROUBLESHOOTING: update called");
            for (size_t i = 0; i < joint_names_.size(); ++i) {
              const auto& joint_name = joint_names_[i];
              if (i < state_interfaces_.size() && i < command_interfaces_.size()) {
                axis_positions_[joint_name] = state_interfaces_[i].get_value();
                command_interfaces_[i].set_value(axis_commands_[joint_name]);
                RCLCPP_DEBUG(rclcpp::get_logger("FourWheelDriveController"),
                             "TROUBLESHOOTING: Joint %s: Position = %.4f, Command = %.4f",
                             joint_name.c_str(), axis_positions_[joint_name],
                             axis_commands_[joint_name]);
              }
            }
            publish_joint_states();
            return controller_interface::return_type::OK;
}

void FourWheelDriveController::cmd_callback(
              const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: cmd_callback called");
            if (msg->data.size() != joint_names_.size()) {
              RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"),
                           "TROUBLESHOOTING: Received command size does not match number of joints");
              return;
            }
            for (size_t i = 0; i < joint_names_.size(); ++i) {
              axis_commands_[joint_names_[i]] = msg->data[i];
              RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                          "TROUBLESHOOTING: Setting %s command to %.4f",
                          joint_names_[i].c_str(), msg->data[i]);
            }
}

void FourWheelDriveController::publish_joint_states() {
            for (const auto& joint : joint_names_) {
              std_msgs::msg::Float64 msg;
              msg.data = axis_positions_[joint];
              joint_state_pubs_[joint]->publish(msg);
              RCLCPP_DEBUG(rclcpp::get_logger("FourWheelDriveController"),
                           "TROUBLESHOOTING: Published position %.4f for joint %s",
                           msg.data, joint.c_str());
            }
}

controller_interface::InterfaceConfiguration
FourWheelDriveController::command_interface_configuration() const {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: command_interface_configuration called");
            controller_interface::InterfaceConfiguration config;
            config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
            for (const auto& joint_name : joint_names_) {
              config.names.push_back(joint_name + "/" +
                                     hardware_interface::HW_IF_POSITION);
              RCLCPP_INFO(
                  rclcpp::get_logger("FourWheelDriveController"),
                  "TROUBLESHOOTING: Added command interface: %s",
                  (joint_name + "/" + hardware_interface::HW_IF_POSITION).c_str());
            }
            return config;
}

controller_interface::InterfaceConfiguration
FourWheelDriveController::state_interface_configuration() const {
            RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                        "TROUBLESHOOTING: state_interface_configuration called");
            controller_interface::InterfaceConfiguration config;
            config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
            for (const auto& joint_name : joint_names_) {
              config.names.push_back(joint_name + "/" +
                                     hardware_interface::HW_IF_POSITION);
              RCLCPP_INFO(
                  rclcpp::get_logger("FourWheelDriveController"),
                  "TROUBLESHOOTING: Added state interface: %s",
                  (joint_name + "/" + hardware_interface::HW_IF_POSITION).c_str());
            }
            return config;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController,
                                 controller_interface::ControllerInterface)
