#include "four_wheel_drive_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController()
    : controller_interface::ControllerInterface() {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][Constructor] Controller instance created");
}

controller_interface::CallbackReturn FourWheelDriveController::on_init() {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][on_init] Initializing controller");
  try {
    auto_declare<std::vector<std::string>>("joints",
                                           std::vector<std::string>());
    auto_declare<double>("wheel_separation", 0.0);
    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("encoder_resolution", 0.0);
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                "[FWD_CTRL][on_init] Parameters declared successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"),
                 "[FWD_CTRL][on_init] Exception during initialization: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][on_configure] Configuring controller");

  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"),
                 "[FWD_CTRL][on_configure] No joints specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  encoder_resolution_ =
      get_node()->get_parameter("encoder_resolution").as_double();

  RCLCPP_INFO(
      rclcpp::get_logger("FourWheelDriveController"),
      "[FWD_CTRL][on_configure] Parameters loaded: wheel_separation=%.2f, "
      "wheel_radius=%.2f, encoder_resolution=%.2f",
      wheel_separation_, wheel_radius_, encoder_resolution_);

  joint_positions_.resize(joint_names_.size(), 0.0);
  joint_velocities_.resize(joint_names_.size(), 0.0);
  joint_commands_.resize(joint_names_.size(), 0.0);

  for (const auto& joint_name : joint_names_) {
    axis_positions_[joint_name] = 0.0;
    axis_commands_[joint_name] = 0.0;
  }

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", 10,
      std::bind(&FourWheelDriveController::cmd_vel_callback, this,
                std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][on_configure] Subscribed to ~/cmd_vel topic");

  cmd_pos_sub_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "~/commands", 10,
          std::bind(&FourWheelDriveController::cmd_pos_callback, this,
                    std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][on_configure] Subscribed to ~/commands topic");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][on_activate] Activating controller");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelDriveController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][on_deactivate] Deactivating controller");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  RCLCPP_DEBUG(rclcpp::get_logger("FourWheelDriveController"),
               "[FWD_CTRL][update] Updating controller state");

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint_name = joint_names_[i];

    // Update position state from encoder
    double encoder_value = state_interfaces_[i].get_value();
    axis_positions_[joint_name] =
        (encoder_value / encoder_resolution_) * 2 * M_PI;

    // Set command
    command_interfaces_[i].set_value(axis_commands_[joint_name]);

    RCLCPP_DEBUG(rclcpp::get_logger("FourWheelDriveController"),
                 "[FWD_CTRL][update] Joint %s: Position = %.4f, Command = %.4f",
                 joint_name.c_str(), axis_positions_[joint_name],
                 axis_commands_[joint_name]);
  }

  return controller_interface::return_type::OK;
}

void FourWheelDriveController::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][cmd_vel_callback] Received cmd_vel: linear.x=%.2f, "
              "angular.z=%.2f",
              msg->linear.x, msg->angular.z);

  double left_velocity =
      (msg->linear.x - msg->angular.z * wheel_separation_ / 2.0) /
      wheel_radius_;
  double right_velocity =
      (msg->linear.x + msg->angular.z * wheel_separation_ / 2.0) /
      wheel_radius_;

  axis_commands_["joint_base_axis_front_left"] = left_velocity;
  axis_commands_["joint_base_axis_front_right"] = right_velocity;
  axis_commands_["joint_base_axis_back_left"] = left_velocity;
  axis_commands_["joint_base_axis_back_right"] = right_velocity;

  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][cmd_vel_callback] Calculated wheel velocities: "
              "left=%.2f, right=%.2f",
              left_velocity, right_velocity);
}

void FourWheelDriveController::cmd_pos_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][cmd_pos_callback] Received position command");

  if (msg->data.size() != joint_names_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("FourWheelDriveController"),
                 "[FWD_CTRL][cmd_pos_callback] Received command size (%zu) "
                 "does not match number of joints (%zu)",
                 msg->data.size(), joint_names_.size());
    return;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_commands_[joint_names_[i]] = msg->data[i];
    RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
                "[FWD_CTRL][cmd_pos_callback] Setting %s position to %.4f",
                joint_names_[i].c_str(), msg->data[i]);
  }
}

controller_interface::InterfaceConfiguration
FourWheelDriveController::command_interface_configuration() const {
  RCLCPP_INFO(rclcpp::get_logger("FourWheelDriveController"),
              "[FWD_CTRL][command_interface_configuration] Configuring command "
              "interfaces");
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" +
                           hardware_interface::HW_IF_POSITION);
  }
  return config;
}
controller_interface::InterfaceConfiguration
FourWheelDriveController::state_interface_configuration() const {
  RCLCPP_INFO(
      rclcpp::get_logger("FourWheelDriveController"),
      "[FWD_CTRL][state_interface_configuration] Configuring state interfaces");
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" +
                           hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" +
                           hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

}  // namespace shelfbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController,
                       controller_interface::ControllerInterface)
