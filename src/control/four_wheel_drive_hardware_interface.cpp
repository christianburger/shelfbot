#include "four_wheel_drive_hardware_interface.hpp"
namespace shelfbot {

FourWheelDriveHardwareInterface::FourWheelDriveHardwareInterface() {
  log_info("FourWheelDriveHardwareInterface", "Constructor", "Initializing hardware interface");
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  log_info("FourWheelDriveHardwareInterface", "on_init", "Starting initialization");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    log_error("FourWheelDriveHardwareInterface", "on_init", "Failed to initialize base class");
    return hardware_interface::CallbackReturn::ERROR;
  }
  log_debug("FourWheelDriveHardwareInterface", "on_init", "Base class initialized successfully");

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  log_info("FourWheelDriveHardwareInterface",
           "on_init",
           "Resized position and command vectors to " + std::to_string(info_.joints.size()));

  for (const auto& joint : info_.joints) {
    log_debug("FourWheelDriveHardwareInterface", "on_init", "Joint found: " + joint.name);
  }

  serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    log_error("FourWheelDriveHardwareInterface", "on_init", "Error opening serial port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  log_info("FourWheelDriveHardwareInterface", "on_init", "Initialization completed successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveHardwareInterface", "on_configure", "Starting configuration");
  log_debug(
      "FourWheelDriveHardwareInterface", "on_configure", "Previous state: " + std::string(previous_state.label()));
  log_info("FourWheelDriveHardwareInterface", "on_configure", "Configuration completed successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelDriveHardwareInterface::export_state_interfaces() {
  log_info("FourWheelDriveHardwareInterface", "export_state_interfaces", "Exporting state interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    log_debug("FourWheelDriveHardwareInterface",
              "export_state_interfaces",
              "Adding state interface for joint: " + info_.joints[i].name);
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  log_info("FourWheelDriveHardwareInterface",
           "export_state_interfaces",
           "Exported " + std::to_string(state_interfaces.size()) + " state interfaces");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelDriveHardwareInterface::export_command_interfaces() {
  log_info("FourWheelDriveHardwareInterface", "export_command_interfaces", "Exporting command interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    log_debug("FourWheelDriveHardwareInterface",
              "export_command_interfaces",
              "Adding command interface for joint: " + info_.joints[i].name);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  log_info("FourWheelDriveHardwareInterface",
           "export_command_interfaces",
           "Exported " + std::to_string(command_interfaces.size()) + " command interfaces");
  return command_interfaces;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveHardwareInterface", "on_activate", "Activating hardware interface");
  log_debug("FourWheelDriveHardwareInterface", "on_activate", "Previous state: " + std::string(previous_state.label()));
  log_info("FourWheelDriveHardwareInterface", "on_activate", "Hardware interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  log_info("FourWheelDriveHardwareInterface", "on_deactivate", "Deactivating hardware interface");
  log_debug(
      "FourWheelDriveHardwareInterface", "on_deactivate", "Previous state: " + std::string(previous_state.label()));
  close(serial_fd_);
  log_info("FourWheelDriveHardwareInterface", "on_deactivate", "Hardware interface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::read(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) {
  log_debug("FourWheelDriveHardwareInterface", "read", "Reading hardware state");
  log_debug("FourWheelDriveHardwareInterface",
            "read",
            "Time: " + std::to_string(time.seconds()) + ", Period: " + std::to_string(period.seconds()));

  if (!readStateFromHardware()) {
    return hardware_interface::return_type::ERROR;
  }

  log_debug("FourWheelDriveHardwareInterface", "read", "Hardware state read successfully");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(const rclcpp::Time& time,
                                                                       const rclcpp::Duration& period) {
  log_debug("FourWheelDriveHardwareInterface", "write", "Writing commands to hardware");
  log_debug("FourWheelDriveHardwareInterface",
            "write",
            "Time: " + std::to_string(time.seconds()) + ", Period: " + std::to_string(period.seconds()));

  if (!writeCommandsToHardware()) {
    return hardware_interface::return_type::ERROR;
  }

  log_debug("FourWheelDriveHardwareInterface", "write", "Commands written to hardware successfully");
  return hardware_interface::return_type::OK;
}

bool FourWheelDriveHardwareInterface::writeCommandsToHardware() {
  msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, hw_commands_);

  ssize_t bytes_written = ::write(serial_fd_, sbuf.data(), sbuf.size());
  if (bytes_written != static_cast<ssize_t>(sbuf.size())) {
    log_error("FourWheelDriveHardwareInterface", "writeCommandsToHardware", "Failed to write to serial port");
    return false;
  }

  std::vector<uint8_t> response(64);
  ssize_t bytes_read = ::read(serial_fd_, response.data(), response.size());
  if (bytes_read <= 0) {
    log_error("FourWheelDriveHardwareInterface", "writeCommandsToHardware", "Failed to read from serial port");
    return false;
  }

  msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<char*>(response.data()), bytes_read);
  msgpack::object obj = oh.get();

  bool success;
  obj.convert(success);
  return success;
}

bool FourWheelDriveHardwareInterface::readStateFromHardware() {
  msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, std::string("position"));

  ssize_t bytes_written = ::write(serial_fd_, sbuf.data(), sbuf.size());
  if (bytes_written != static_cast<ssize_t>(sbuf.size())) {
    log_error("FourWheelDriveHardwareInterface", "readStateFromHardware", "Failed to write to serial port");
    return false;
  }

  std::vector<uint8_t> response(64);
  ssize_t bytes_read = ::read(serial_fd_, response.data(), response.size());
  if (bytes_read <= 0) {
    log_error("FourWheelDriveHardwareInterface", "readStateFromHardware", "Failed to read from serial port");
    return false;
  }

  msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<char*>(response.data()), bytes_read);
  msgpack::object obj = oh.get();

  obj.convert(hw_positions_);
  return true;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)
