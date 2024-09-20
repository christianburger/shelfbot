#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace four_wheel_drive_controller
{

class FourWheelDriveController : public controller_interface::ControllerInterface
{
public:
  FourWheelDriveController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void updateOdometry(const rclcpp::Time & time);
  void publishOdometry(const rclcpp::Time & time);
  std::vector<double> calculateWheelSpeeds(double linear_x, double linear_y, double angular_z);

  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
  };

  std::vector<WheelHandle> wheel_handles_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> received_velocity_msg_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::vector<std::string> wheel_names_;
  std::vector<std::string> base_axis_names_;
  std::vector<std::string> motor_names_;
  double wheel_separation_width_;
  double wheel_separation_length_;
  double wheel_radius_;
  int encoder_resolution_;
  bool use_sim_encoders_;
  double publish_rate_;
  bool enable_odom_tf_;
  std::chrono::milliseconds cmd_vel_timeout_;

  // Odometry related variables
  double x_;
  double y_;
  double theta_;  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::vector<std::string> wheel_names_;
  std::vector<std::string> base_axis_names_;
  std::vector<std::string> motor_names_;
  double wheel_separation_width_;
  double wheel_separation_length_;
  double wheel_radius_;
  int encoder_resolution_;
  bool use_sim_encoders_;
  double publish_rate_;
  bool enable_odom_tf_;
  std::chrono::milliseconds cmd_vel_timeout_;

  // Odometry related variables
  double x_;
  double y_;
  double theta_;
};

}  // namespace four_wheel_drive_controller

#endif  // FOUR_WHEEL_DRIVE_CONTROLLER_HPP
