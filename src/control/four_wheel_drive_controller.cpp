#include "four_wheel_drive_controller.hpp"

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using std::string;
using std::to_string;
using std::vector;
using std::placeholders::_1;

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController() : controller_interface::ControllerInterface(), clock_(RCL_SYSTEM_TIME) {
  log_info("FourWheelDriveController", "Constructor", "Called");
  for (const auto& joint : joint_names_) {
    log_debug("FourWheelDriveController", "Constructor", "Joint name: " + joint);
  }

}
CallbackReturn FourWheelDriveController::on_init() {
  log_info("FourWheelDriveController", "on_init", "Called");
  try {
    auto_declare<vector<string>>("joints", vector<string>());
    auto_declare<double>("wheel_separation", 0.0);
    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("base_height", 0.0);
    log_info("FourWheelDriveController", "on_init", "Parameters declared");

  } catch (const std::exception& e) {
    log_error("FourWheelDriveController", "on_init", string("Exception: ") + e.what());
    return CallbackReturn::ERROR;
  }
  log_info("FourWheelDriveController", "on_init", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveController", "on_configure", "Starting configuration");
 
  // Initialize the clock
  clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);

  joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  base_height_ = get_node()->get_parameter("base_height").as_double();
  if (joint_names_.empty()) {
    log_error("FourWheelDriveController", "on_configure", "No joints specified");
    return CallbackReturn::ERROR;
  }

  log_info("FourWheelDriveController", "on_configure", "Joints loaded: " + to_string(joint_names_.size()));

  for (const auto& joint : joint_names_) {
    axis_positions_[joint] = 0.0;
    axis_commands_[joint] = 0.0;
    log_info("FourWheelDriveController", "on_configure", "Initialized joint: " + joint);

    joint_state_pubs_[joint] = get_node()->create_publisher<std_msgs::msg::Float64>(joint + "/position", 10);
    log_info("FourWheelDriveController", "on_configure", "Created publisher for joint: " + joint);
  }

  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", 10, std::bind(&FourWheelDriveController::cmd_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

  log_info("FourWheelDriveController", "on_configure", "Joint state broadcaster initialization check");
  auto js_broadcaster = get_node()->get_parameter("use_joint_state_broadcaster").as_bool();
  if (js_broadcaster) {
    log_info("FourWheelDriveController", "on_configure", "Joint state broadcaster is enabled");
  } else {
    log_debug("FourWheelDriveController", "on_configure", "Joint state broadcaster is disabled");
  }
  
  for (const auto& joint : joint_names_) {
    log_debug("FourWheelDriveController", "on_configure", "Configured joint: " + joint);
  }

  // In on_configure():
  init_odometry();

  log_info("FourWheelDriveController", "on_configure", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveController", "on_activate", "Starting activation process");
  log_info("FourWheelDriveController", "on_activate", "Previous state: " + std::string(previous_state.label()));
  log_info("FourWheelDriveController", "on_activate", "Activation process completed");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveController", "on_deactivate", "Called");
  log_info("FourWheelDriveController", "on_deactivate", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  log_debug("FourWheelDriveController", "update", "Called");

  // Read current joint states from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_positions_[joint_names_[i]] = state_interfaces_[i].get_value();
  }

  update_odometry(period);
  // Publish joint states and transforms
  publish_joint_states();

  // Write commands to command interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[i].set_value(axis_commands_[joint_names_[i]]);
  }

  return controller_interface::return_type::OK;
}

void FourWheelDriveController::cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  log_info("FourWheelDriveController", "cmd_callback", "Called");
  if (msg->data.size() != joint_names_.size()) {
    log_error("FourWheelDriveController", "cmd_callback", "Received command size does not match number of joints");
    return;
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_commands_[joint_names_[i]] = msg->data[i];
    log_info("FourWheelDriveController",
             "cmd_callback",
             "Setting " + joint_names_[i] + " command to " + std::to_string(msg->data[i]));
  }
}

void FourWheelDriveController::publish_joint_states() {
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = clock_.now();
  joint_state_msg.header.frame_id = "base_link";

  log_debug("FourWheelDriveController", "publish_joint_states", "Publishing joint states");

  for (const auto& joint : joint_names_) {
    joint_state_msg.name.push_back(joint);
    joint_state_msg.position.push_back(axis_positions_[joint]);

    log_debug("FourWheelDriveController",
              "publish_joint_states",
              "Joint: " + joint + ", Position: " + std::to_string(axis_positions_[joint]));
  }

  joint_state_publisher_->publish(joint_state_msg);

  log_debug("FourWheelDriveController",
            "publish_joint_states",
            "Published joint states for " + std::to_string(joint_names_.size()) + " joints");
}

InterfaceConfiguration FourWheelDriveController::command_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

InterfaceConfiguration FourWheelDriveController::state_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

void FourWheelDriveController::init_odometry() {
    odometry_ = std::make_unique<FourWheelDriveOdometry>(
        get_node(),
        std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME),
        wheel_separation_,
        wheel_radius_
    );
    log_info("FourWheelDriveController", "init_odometry", "Odometry initialized");
}

void FourWheelDriveController::update_odometry(const rclcpp::Duration& period) {
    std::vector<double> wheel_positions;
    for (const auto& joint : joint_names_) {
        wheel_positions.push_back(axis_positions_[joint]);
    }
    odometry_->update(wheel_positions, period);
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)
