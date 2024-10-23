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

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

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

  // Read current joint states from state interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    axis_positions_[joint_names_[i]] = state_interfaces_[i].get_value();
  }

  // Update odometry
  update_odometry(period);

  // Publish joint states and transforms
  publish_joint_states();
  publish_transforms();

  // Write commands to command interfaces
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[i].set_value(axis_commands_[joint_names_[i]]);
  }

  return controller_interface::return_type::OK;
}

void FourWheelDriveController::update_odometry(const rclcpp::Duration& period) {
  // Calculate wheel velocities
  std::vector<double> wheel_velocities(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    wheel_velocities[i] = axis_positions_[joint_names_[i]] / period.seconds();
  }

  // Calculate robot velocity
  double vx = std::accumulate(wheel_velocities.begin(), wheel_velocities.end(), 0.0) * wheel_radius_ / joint_names_.size();
  double vy = 0.0;  // Assuming no lateral movement
  double vth = (wheel_velocities[1] + wheel_velocities[3] - wheel_velocities[0] - wheel_velocities[2]) * wheel_radius_ / (2.0 * wheel_separation_);

  // Update position
  double delta_x = (vx * std::cos(theta_) - vy * std::sin(theta_)) * period.seconds();
  double delta_y = (vx * std::sin(theta_) + vy * std::cos(theta_)) * period.seconds();
  double delta_th = vth * period.seconds();

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_th;

  // Create and publish odometry message
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->header.stamp = clock_.now();
  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_link";
  odom_msg->pose.pose.position.x = x_;
  odom_msg->pose.pose.position.y = y_;
  odom_msg->pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(theta_ / 2), std::cos(theta_ / 2)));
  odom_msg->twist.twist.linear.x = vx;
  odom_msg->twist.twist.linear.y = vy;
  odom_msg->twist.twist.angular.z = vth;
  odom_pub_->publish(std::move(odom_msg));
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

void FourWheelDriveController::publish_transforms() {
  log_debug("FourWheelDriveController", "publish_transforms", "Starting transform publication");

  auto now = get_node()->now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // Only publish the dynamic transforms from motors to base_axis
  for (const auto& joint : joint_names_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    
    // Extract the corresponding motor name from the joint name
    std::string motor_name = "motor_" + joint.substr(joint.find("base_axis_") + 10);
    std::string base_axis_name = "base_axis_" + joint.substr(joint.find("base_axis_") + 10);
    
    transform.header.frame_id = motor_name;
    transform.child_frame_id = base_axis_name;
    transform.transform.translation.x = 0.1;  // motor_length/2
    
    // Set rotation based on joint position
    tf2::Quaternion q;
    q.setRPY(0, 0, axis_positions_[joint]);
    transform.transform.rotation = tf2::toMsg(q);

    transforms.push_back(transform);
    log_debug("FourWheelDriveController", "publish_transforms", 
              "Published transform from " + motor_name + " to " + base_axis_name);
  }

  tf_broadcaster_->sendTransform(transforms);
  log_debug("FourWheelDriveController", "publish_transforms", 
            "Published " + std::to_string(transforms.size()) + " transforms");
}

geometry_msgs::msg::TransformStamped FourWheelDriveController::create_transform(
    const std::string& frame_id, const std::string& child_frame_id,
    double x, double y, double z, double roll, double pitch, double yaw,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = frame_id;
  t.child_frame_id = child_frame_id;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  t.transform.rotation = tf2::toMsg(q);
  return t;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveController, controller_interface::ControllerInterface)
