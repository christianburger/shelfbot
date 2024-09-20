
#include "include/four_wheel_drive_controller.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

namespace four_wheel_drive_controller
{

FourWheelDriveController::FourWheelDriveController() : controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn FourWheelDriveController::on_init()
{
  auto node = get_node();

  // Get parameters
  node->get_parameter("wheel_names", wheel_names_);
  node->get_parameter("base_axis_names", base_axis_names_);
  node->get_parameter("motor_names", motor_names_);
  node->get_parameter("wheel_separation_width", wheel_separation_width_);
  node->get_parameter("wheel_separation_length", wheel_separation_length_);
  node->get_parameter("wheel_radius", wheel_radius_);
  node->get_parameter("encoder_resolution", encoder_resolution_);
  node->get_parameter("use_sim_encoders", use_sim_encoders_);
  node->get_parameter("publish_rate", publish_rate_);
  node->get_parameter("enable_odom_tf", enable_odom_tf_);

  double cmd_vel_timeout_sec;
  node->get_parameter("cmd_vel_timeout", cmd_vel_timeout_sec);
  cmd_vel_timeout_ = std::chrono::milliseconds(static_cast<int>(cmd_vel_timeout_sec * 1000.0));

  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // Initialize odometry
  x_ = y_ = theta_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FourWheelDriveController::command_interface_configuration() const
{
  std::vector<std::string> command_interfaces;
  for (const auto & wheel : wheel_names_)
  {
    command_interfaces.push_back(wheel + "/velocity");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces};
}

controller_interface::InterfaceConfiguration FourWheelDriveController::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces;
  for (const auto & wheel : wheel_names_)
  {
    state_interfaces.push_back(wheel + "/position");
    state_interfaces.push_back(wheel + "/velocity");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces};
}

controller_interface::return_type FourWheelDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();

  // Get the latest velocity command
  geometry_msgs::msg::Twist twist_msg = received_velocity_msg_.readFromRT();

  // Calculate wheel speeds
  auto wheel_speeds = calculateWheelSpeeds(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);

  // Set wheel speeds
  for (size_t i = 0; i < wheel_handles_.size(); ++i)
  {
    wheel_handles_[i].velocity_command.get().set_value(wheel_speeds[i]);
  }

  // Update odometry
  updateOdometry(time);

  // Publish odometry
  publishOdometry(time);

  return controller_interface::return_type::OK;
}

void FourWheelDriveController::updateOdometry(const rclcpp::Time & time)
{
  double dt = (time - last_update_time_).seconds();
  last_update_time_ = time;

  double left_wheel_vel = (wheel_handles_[0].velocity.get().get_value() + wheel_handles_[2].velocity.get().get_value()) / 2.0;
  double right_wheel_vel = (wheel_handles_[1].velocity.get().get_value() + wheel_handles_[3].velocity.get().get_value()) / 2.0;

  double linear_vel = (right_wheel_vel + left_wheel_vel) * wheel_radius_ / 2.0;
  double angular_vel = (right_wheel_vel - left_wheel_vel) * wheel_radius_ / wheel_separation_width_;

  double delta_x = linear_vel * std::cos(theta_) * dt;
  double delta_y = linear_vel * std::sin(theta_) * dt;
  double delta_theta = angular_vel * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;
}

void FourWheelDriveController::publishOdometry(const rclcpp::Time & time)
{
  if (realtime_odometry_publisher_ && realtime_odometry_publisher_->trylock())
  {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = time;
    odometry_message.pose.pose.position.x = x_;
    odometry_message.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odometry_message.pose.pose.orientation.x = q.x();
    odometry_message.pose.pose.orientation.y = q.y();
    odometry_message.pose.pose.orientation.z = q.z();
    odometry_message.pose.pose.orientation.w = q.w();

    realtime_odometry_publisher_->unlockAndPublish();
  }

  if (enable_odom_tf_ && realtime_odometry_transform_publisher_ && realtime_odometry_transform_publisher_->trylock())
  {
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = time;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    realtime_odometry_transform_publisher_->unlockAndPublish();
  }
}

std::vector<double> FourWheelDriveController::calculateWheelSpeeds(double linear_x, double linear_y, double angular_z)
{
  std::vector<double> wheel_speeds(4);

  double vx = linear_x;
  double vy = linear_y;
  double vw = angular_z;

  wheel_speeds[0] = (vx - vy - (wheel_separation_width_ + wheel_separation_length_) * vw) / wheel_radius_;  // Front left
  wheel_speeds[1] = (vx + vy + (wheel_separation_width_ + wheel_separation_length_) * vw) / wheel_radius_;  // Front right
  wheel_speeds[2] = (vx + vy - (wheel_separation_width_ + wheel_separation_length_) * vw) / wheel_radius_;  // Rear left
  wheel_speeds[3] = (vx - vy + (wheel_separation_width_ + wheel_separation_length_) * vw) / wheel_radius_;  // Rear right

  return wheel_speeds;
}

}  // namespace four_wheel_drive_controller

#include "class_loader/register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(four_wheel_drive_controller::FourWheelDriveController, controller_interface::ControllerInterface)
