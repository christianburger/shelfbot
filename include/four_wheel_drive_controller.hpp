#ifndef FOUR_WHEEL_DRIVE_CONTROLLER_HPP
#define FOUR_WHEEL_DRIVE_CONTROLLER_HPP

#include <tf2_ros/transform_broadcaster.h>
#include <functional>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shelfbot_utils.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace shelfbot {

class FourWheelDriveController : public controller_interface::ControllerInterface {
 public:
  FourWheelDriveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> axis_positions_;
  std::map<std::string, double> axis_commands_;

  double wheel_separation_;
  double wheel_radius_;
  double base_height_;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
  double prev_left_wheel_pos_ = 0.0;
  double prev_right_wheel_pos_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_state_pubs_;

  void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_joint_states();
  void publish_transforms();
  void update_odometry(const rclcpp::Duration& period);
  private:
  rclcpp::Clock clock_;
  geometry_msgs::msg::TransformStamped create_transform(
    const std::string& frame_id, const std::string& child_frame_id,
    double x, double y, double z, double roll, double pitch, double yaw,
    const rclcpp::Time& stamp);
};
} 

#endif
