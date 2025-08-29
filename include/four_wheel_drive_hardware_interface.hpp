#ifndef FOUR_WHEEL_DRIVE_HARDWARE_INTERFACE_HPP
#define FOUR_WHEEL_DRIVE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shelfbot_utils.hpp"
#include "communication_interface.hpp"
#include "four_wheel_drive_odometry.hpp" // Include the odometry header
#include <rclcpp/logging.hpp>
#include <thread>

namespace shelfbot {

class FourWheelDriveHardwareInterface : public hardware_interface::SystemInterface {
 public:
  FourWheelDriveHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::thread node_spinner_;
  std::unique_ptr<CommunicationInterface> comm_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_max_speeds_;
  
  // Odometry moved to the hardware interface
  std::unique_ptr<FourWheelDriveOdometry> odometry_;
};

}
#endif