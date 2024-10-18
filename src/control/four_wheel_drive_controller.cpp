#include "four_wheel_drive_controller.hpp"

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using std::string;
using std::to_string;
using std::vector;
using std::placeholders::_1;

namespace shelfbot {

FourWheelDriveController::FourWheelDriveController()
: ControllerInterface() {
  log_info("FourWheelDriveController", "Constructor", "Called");
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

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

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

controller_interface::return_type FourWheelDriveController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period) {
  log_debug("FourWheelDriveController", "update", "Called");
  double left_wheel_pos = 0.0, right_wheel_pos = 0.0;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& joint_name = joint_names_[i];
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

    command_interfaces_[i].set_value(axis_commands_[joint_name]);

    log_debug("FourWheelDriveController",
              "update",
              "Joint " + joint_name + ": Command = " + to_string(axis_commands_[joint_name]));
  }

  left_wheel_pos /= 2.0;
  right_wheel_pos /= 2.0;

  double left_wheel_delta = left_wheel_pos - prev_left_wheel_pos_;
  double right_wheel_delta = right_wheel_pos - prev_right_wheel_pos_;

  double linear_delta = (left_wheel_delta + right_wheel_delta) * wheel_radius_ / 2.0;
  double angular_delta = (right_wheel_delta - left_wheel_delta) * wheel_radius_ / wheel_separation_;

  x_ += linear_delta * cos(theta_);
  y_ += linear_delta * sin(theta_);
  theta_ += angular_delta;

  prev_left_wheel_pos_ = left_wheel_pos;
  prev_right_wheel_pos_ = right_wheel_pos;

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
  log_debug("FourWheelDriveController", "update", "Joint states published");
  publish_transforms();
  log_debug("FourWheelDriveController", "update", "Transforms published");

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
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = get_node()->now();
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

void FourWheelDriveController::publish_transforms() {
  log_debug("FourWheelDriveController", "publish_transforms", "Starting transform publication");

  auto now = get_node()->now();

  // Publish base_footprint to base_link transform
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "base_footprint";
  t.child_frame_id = "base_link";
  t.transform.translation.z = wheel_radius_;
  t.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(t);
  log_debug("FourWheelDriveController", "publish_transforms", "Published base_footprint to base_link transform");

  // Publish transforms for base_axis joints and wheels
  for (const auto& joint : joint_names_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = joint;

    // Set translation based on joint position
    if (joint.find("front_left") != std::string::npos) {
      transform.transform.translation.x = -wheel_separation_ / 2;
      transform.transform.translation.y = wheel_separation_ / 2;
    } else if (joint.find("front_right") != std::string::npos) {
      transform.transform.translation.x = wheel_separation_ / 2;
      transform.transform.translation.y = wheel_separation_ / 2;
    } else if (joint.find("back_left") != std::string::npos) {
      transform.transform.translation.x = -wheel_separation_ / 2;
      transform.transform.translation.y = -wheel_separation_ / 2;
    } else if (joint.find("back_right") != std::string::npos) {
      transform.transform.translation.x = wheel_separation_ / 2;
      transform.transform.translation.y = -wheel_separation_ / 2;
    }

    // Set rotation based on joint position
    tf2::Quaternion q;
    q.setRPY(0, 0, axis_positions_[joint]);
    transform.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(transform);
    log_debug("FourWheelDriveController", "publish_transforms", "Published transform for joint: " + joint);

    // Publish wheel transform
    geometry_msgs::msg::TransformStamped wheel_transform;
    wheel_transform.header.stamp = now;
    wheel_transform.header.frame_id = joint;
    wheel_transform.child_frame_id = "wheel_" + joint.substr(joint.find_last_of("_") + 1);
    wheel_transform.transform.translation.x = wheel_radius_;
    wheel_transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(wheel_transform);
    log_debug("FourWheelDriveController", "publish_transforms", "Published wheel transform for: " + wheel_transform.child_frame_id);
  }

  // Publish indicator_box transform
  geometry_msgs::msg::TransformStamped indicator_transform;
  indicator_transform.header.stamp = now;
  indicator_transform.header.frame_id = "base_link";
  indicator_transform.child_frame_id = "indicator_box";
  indicator_transform.transform.translation.z = base_height_ / 2;
  indicator_transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(indicator_transform);
  log_debug("FourWheelDriveController", "publish_transforms", "Published indicator_box transform");

  log_debug("FourWheelDriveController", "publish_transforms", "Completed transform publication");
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)