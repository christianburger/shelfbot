#include "four_wheel_drive_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <functional>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using std::string;
using std::to_string;
using std::vector;
using std::placeholders::_1;

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController() : ControllerInterface() {
  log_info("FourWheelDriveController", "Constructor", "Called");
}

CallbackReturn FourWheelDriveController::on_init() {
  log_info("FourWheelDriveController", "on_init", "Called");
  try {
    auto_declare<vector<string>>("joints", vector<string>());
    auto_declare<double>("wheel_separation", 0.0);
    auto_declare<double>("wheel_radius", 0.0);
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
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
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

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  log_info("FourWheelDriveController", "on_configure", "Joint state broadcaster initialization check");
  auto js_broadcaster = get_node()->get_parameter("use_joint_state_broadcaster").as_bool();
  if (js_broadcaster) {
    log_info("FourWheelDriveController", "on_configure", "Joint state broadcaster is enabled");
  } else {
    log_debug("FourWheelDriveController", "on_configure", "Joint state broadcaster is disabled");
  }

  log_info("FourWheelDriveController", "on_configure", "Completed successfully");
  return CallbackReturn::SUCCESS;
}
CallbackReturn FourWheelDriveController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveController", "on_activate", "Called");
  log_info("FourWheelDriveController", "on_activate", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FourWheelDriveController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveController", "on_deactivate", "Called");
  log_info("FourWheelDriveController", "on_deactivate", "Completed successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelDriveController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period) {
  log_debug("FourWheelDriveController", "update", "Called");
  double left_wheel_pos = 0.0, right_wheel_pos = 0.0;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint_name = joint_names_[i];
    if (i < state_interfaces_.size()) {
      double current_position = state_interfaces_[i].get_value();
      if (joint_name.find("left") != std::string::npos) {
        left_wheel_pos += current_position;
      } else if (joint_name.find("right") != std::string::npos) {
        right_wheel_pos += current_position;
      }
      axis_positions_[joint_name] = current_position;

      log_debug("FourWheelDriveController",
                "update",
                "Joint " + joint_name + ": Position = " + to_string(current_position) +
                    ", Previous = " + to_string(axis_positions_[joint_name]));

      if (i < command_interfaces_.size()) {
        command_interfaces_[i].set_value(axis_commands_[joint_name]);
        log_debug("FourWheelDriveController",
                  "update",
                  "Joint " + joint_name + ": Command = " + to_string(axis_commands_[joint_name]));
      }
    }
  }

  // Calculate odometry
  left_wheel_pos /= 2.0;   // Average of left wheels
  right_wheel_pos /= 2.0;  // Average of right wheels
  double left_wheel_delta = left_wheel_pos - prev_left_wheel_pos_;
  double right_wheel_delta = right_wheel_pos - prev_right_wheel_pos_;

  double linear_delta = (left_wheel_delta + right_wheel_delta) * wheel_radius_ / 2.0;
  double angular_delta = (right_wheel_delta - left_wheel_delta) * wheel_radius_ / wheel_separation_;

  x_ += linear_delta * cos(theta_);
  y_ += linear_delta * sin(theta_);
  theta_ += angular_delta;

  prev_left_wheel_pos_ = left_wheel_pos;
  prev_right_wheel_pos_ = right_wheel_pos;

  // Publish odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = get_node()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(theta_ / 2), std::cos(theta_ / 2)));
  odom_msg.twist.twist.linear.x = linear_delta / period.seconds();
  odom_msg.twist.twist.angular.z = angular_delta / period.seconds();
  odom_pub_->publish(odom_msg);

  publish_joint_states();
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
             "Setting " + joint_names_[i] + " command to " + to_string(msg->data[i]));
  }
}

void FourWheelDriveController::publish_joint_states() {
  for (const auto& joint : joint_names_) {
    std_msgs::msg::Float64 msg;
    msg.data = axis_positions_[joint];
    joint_state_pubs_[joint]->publish(msg);
    log_debug("FourWheelDriveController",
              "publish_joint_states",
              "Published position " + to_string(msg.data) + " for joint " + joint);
  }
}

InterfaceConfiguration FourWheelDriveController::command_interface_configuration() const {
  log_info("FourWheelDriveController", "command_interface_configuration", "Called");
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    log_info("FourWheelDriveController",
             "command_interface_configuration",
             "Added command interface: " + joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

InterfaceConfiguration FourWheelDriveController::state_interface_configuration() const {
  log_info("FourWheelDriveController", "state_interface_configuration", "Called");
  InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    log_info("FourWheelDriveController",
             "state_interface_configuration",
             "Added state interface: " + joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

bool FourWheelDriveController::writeCommandsToHardware(const std::vector<double>& wheel_positions) {
  msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, wheel_positions);

  ssize_t bytes_written = write(serial_fd_, sbuf.data(), sbuf.size());
  if (bytes_written != static_cast<ssize_t>(sbuf.size())) {
    log_error("FourWheelDriveController", "writeCommandsToHardware", "Failed to write to serial port");
    return false;
  }

  std::vector<uint8_t> response(64);
  ssize_t bytes_read = read(serial_fd_, response.data(), response.size());
  if (bytes_read <= 0) {
    log_error("FourWheelDriveController", "writeCommandsToHardware", "Failed to read from serial port");
    return false;
  }

  msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<char*>(response.data()), bytes_read);
  msgpack::object obj = oh.get();

  bool success;
  obj.convert(success);
  return success;
}

std::vector<double> FourWheelDriveController::readStateFromHardware(const std::string& value_type) {
  msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, value_type);

  ssize_t bytes_written = write(serial_fd_, sbuf.data(), sbuf.size());
  if (bytes_written != static_cast<ssize_t>(sbuf.size())) {
    log_error("FourWheelDriveController", "readStateFromHardware", "Failed to write to serial port");
    return {};
  }

  std::vector<uint8_t> response(64);
  ssize_t bytes_read = read(serial_fd_, response.data(), response.size());
  if (bytes_read <= 0) {
    log_error("FourWheelDriveController", "readStateFromHardware", "Failed to read from serial port");
    return {};
  }

  msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<char*>(response.data()), bytes_read);
  msgpack::object obj = oh.get();

  std::vector<double> wheel_positions;
  obj.convert(wheel_positions);
  return wheel_positions;
}
}  // namespace shelfbot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)
