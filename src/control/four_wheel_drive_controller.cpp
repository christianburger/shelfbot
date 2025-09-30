#include "shelfbot/four_wheel_drive_controller.hpp"

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using std::string;
using std::vector;

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController()
: controller_interface::ControllerInterface(),
  cmd_vel_timeout_(0, 0)
{
}

CallbackReturn FourWheelDriveController::on_init() {
  try {
    auto_declare<vector<string>>("joint_names", vector<string>());
    auto_declare<vector<string>>("front_left_joint_names", vector<string>());
    auto_declare<vector<string>>("back_left_joint_names", vector<string>());
    auto_declare<vector<string>>("front_right_joint_names", vector<string>());
    auto_declare<vector<string>>("back_right_joint_names", vector<string>());
    auto_declare<double>("wheel_separation", 0.0);
    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("cmd_vel_timeout", 0.5);
  } catch (const std::exception& e) {
    log_error("FourWheelDriveController", "on_init", string("Exception: ") + e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  front_left_joint_names_ = get_node()->get_parameter("front_left_joint_names").as_string_array();
  back_left_joint_names_ = get_node()->get_parameter("back_left_joint_names").as_string_array();
  front_right_joint_names_ = get_node()->get_parameter("front_right_joint_names").as_string_array();
  back_right_joint_names_ = get_node()->get_parameter("back_right_joint_names").as_string_array();
  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(get_node()->get_parameter("cmd_vel_timeout").as_double());

  for (const auto& joint : joint_names_) {
    axis_positions_[joint] = 0.0;
    axis_commands_[joint] = 0.0;
  }

  // FIX: Use SystemDefaultsQoS to match Nav2 controller server publisher
  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&FourWheelDriveController::cmd_vel_callback, this, std::placeholders::_1));

  direct_cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/direct_commands", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == joint_names_.size()) {
          for (size_t i = 0; i < joint_names_.size(); ++i) {
            axis_commands_[joint_names_[i]] = msg->data[i];
          }
          last_cmd_vel_ = nullptr;
        }
      });

  joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_commands_[joint_names_[i]] = state_interfaces_[i].get_value();
  }
  last_cmd_vel_ = nullptr;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Read current positions from hardware
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_positions_[joint_names_[i]] = state_interfaces_[i].get_value();
  }

  // Diagnostic logging for position readings
  static auto last_position_log = time;
  if ((time - last_position_log).seconds() > 2.0) {
    std::stringstream ss;
    ss << "[NAV2_DIAG] Controller read positions: [";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      ss << joint_names_[i] << "=" << axis_positions_[joint_names_[i]];
      if (i < joint_names_.size() - 1) ss << ", ";
    }
    ss << "] - Period: " << period.seconds() << "s";
    log_info("FourWheelDriveController", "update", ss.str());
    last_position_log = time;
  }

  // Check for cmd_vel timeout and process commands
  bool cmd_vel_valid = false;
  double linear_vel = 0.0, angular_vel = 0.0;
  
  if (last_cmd_vel_ && (time - last_cmd_vel_time_) < cmd_vel_timeout_) {
    cmd_vel_valid = true;
    linear_vel = last_cmd_vel_->linear.x;
    angular_vel = last_cmd_vel_->angular.z;
    
    // Calculate wheel velocities
    double vel_front_left  = (linear_vel + angular_vel * wheel_separation_ / 2.0) / wheel_radius_;
    double vel_front_right = (linear_vel - angular_vel * wheel_separation_ / 2.0) / wheel_radius_;
    double vel_back_left   = (-linear_vel + angular_vel * wheel_separation_ / 2.0) / wheel_radius_;
    double vel_back_right  = (-linear_vel - angular_vel * wheel_separation_ / 2.0) / wheel_radius_;

    // Apply calculated velocities
    axis_commands_[front_left_joint_names_[0]] = vel_front_left;
    axis_commands_[front_right_joint_names_[0]] = vel_front_right;
    axis_commands_[back_left_joint_names_[0]] = vel_back_left;
    axis_commands_[back_right_joint_names_[0]] = vel_back_right;
    
    // Enhanced diagnostic logging for command processing
    static auto last_cmd_log = time;
    if ((time - last_cmd_log).seconds() > 1.0) {
      std::stringstream cmd_ss;
      cmd_ss << std::fixed << std::setprecision(3)
             << "[NAV2_DIAG] Cmd_vel received: linear=" << linear_vel 
             << " angular=" << angular_vel 
             << " age=" << (time - last_cmd_vel_time_).seconds() << "s"
             << " → wheel_vels: FL=" << vel_front_left
             << " FR=" << vel_front_right 
             << " BL=" << vel_back_left 
             << " BR=" << vel_back_right;
      log_info("FourWheelDriveController", "update", cmd_ss.str());
      last_cmd_log = time;
    }
  } else {
    // Timeout or no command - zero all velocities
    for (const auto& joint : joint_names_) {
      axis_commands_[joint] = 0.0;
    }
    
    // Log timeout condition
    static auto last_timeout_log = time;
    if ((time - last_timeout_log).seconds() > 3.0) {
      std::string timeout_reason;
      if (!last_cmd_vel_) {
        timeout_reason = "No cmd_vel received yet";
      } else {
        double cmd_age = (time - last_cmd_vel_time_).seconds();
        timeout_reason = "cmd_vel timeout: age=" + std::to_string(cmd_age) + 
                        "s (limit=" + std::to_string(cmd_vel_timeout_.seconds()) + "s)";
      }
      log_warn("FourWheelDriveController", "update", 
               "[NAV2_DIAG] Setting zero velocities - " + timeout_reason);
      last_timeout_log = time;
    }
  }

  // Write commands to hardware interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[i].set_value(axis_commands_[joint_names_[i]]);
  }

  // Enhanced diagnostic logging for command output
  static auto last_output_log = time;
  if ((time - last_output_log).seconds() > 1.0) {
    std::stringstream out_ss;
    out_ss << std::fixed << std::setprecision(3)
           << "[NAV2_DIAG] Commands sent to hardware: [";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      out_ss << joint_names_[i] << "=" << axis_commands_[joint_names_[i]];
      if (i < joint_names_.size() - 1) out_ss << ", ";
    }
    out_ss << "] Valid_cmd=" << (cmd_vel_valid ? "Yes" : "No");
    log_info("FourWheelDriveController", "update", out_ss.str());
    last_output_log = time;
  }

  // Publish joint states with enhanced diagnostics
  publish_joint_states();

  // Overall system health check
  static auto last_health_log = time;
  if ((time - last_health_log).seconds() > 5.0) {
    std::stringstream health_ss;
    health_ss << "[NAV2_DIAG] Controller Health: "
              << "Joints=" << joint_names_.size()
              << " Period=" << std::fixed << std::setprecision(3) << period.seconds() << "s"
              << " CmdVel=" << (cmd_vel_valid ? "ACTIVE" : "TIMEOUT")
              << " LastCmd=" << (last_cmd_vel_ ? 
                  std::to_string((time - last_cmd_vel_time_).seconds()) + "s ago" : "NEVER");
    log_info("FourWheelDriveController", "update", health_ss.str());
    last_health_log = time;
  }

  return controller_interface::return_type::OK;
}

void FourWheelDriveController::cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
  last_cmd_vel_ = msg;
  last_cmd_vel_time_ = get_node()->now();
  
  // Enhanced cmd_vel reception logging
  static int cmd_count = 0;
  static auto last_callback_log = get_node()->now();
  
  cmd_count++;
  if ((get_node()->now() - last_callback_log).seconds() > 2.0) {
    std::stringstream cb_ss;
    cb_ss << std::fixed << std::setprecision(3)
          << "[NAV2_DIAG] Cmd_vel callback: count=" << cmd_count 
          << " rate=" << (cmd_count / 2.0) << "Hz"
          << " linear.x=" << msg->linear.x 
          << " angular.z=" << msg->angular.z
          << " stamp=" << get_node()->now().seconds();
    log_info("FourWheelDriveController", "cmd_vel_callback", cb_ss.str());
    cmd_count = 0;
    last_callback_log = get_node()->now();
  }
}

void FourWheelDriveController::publish_joint_states() {
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = get_node()->now();
  joint_state_msg.header.frame_id = "base_link";

  for (const auto& joint : joint_names_) {
    joint_state_msg.name.push_back(joint);
    joint_state_msg.position.push_back(axis_positions_[joint]);
  }

  std::stringstream ss;
  ss << "[hw_positions] FourWheelDriveController::publish_joint_states: Publishing positions: [";
  for (size_t i = 0; i < joint_state_msg.position.size(); ++i) {
      ss << joint_state_msg.position[i] << (i < joint_state_msg.position.size() - 1 ? ", " : "");
  }
  ss << "]";
  log_info("FourWheelDriveController", "publish_joint_states", ss.str());

  joint_state_publisher_->publish(joint_state_msg);
}

InterfaceConfiguration FourWheelDriveController::command_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
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

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)
