#include "four_wheel_drive_hardware_interface.hpp"
#include "rest_communication.hpp"
#include "mock_communication.hpp"
#include "microros_communication.hpp"

namespace shelfbot {

FourWheelDriveHardwareInterface::FourWheelDriveHardwareInterface() {
    log_info("FourWheelDriveHardwareInterface", "Constructor", "Starting hardware interface initialization");
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    log_info("FourWheelDriveHardwareInterface", "on_init", "Starting initialization sequence");
    
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        log_error("FourWheelDriveHardwareInterface", "on_init", "Base initialization failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize storage
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_max_speeds_.resize(info_.joints.size(), 10.0); // Default to 10 rad/s

    // Get communication type parameter
    std::string comm_type = info_.hardware_parameters.at("communication_type");
    log_info("FourWheelDriveHardwareInterface", "on_init", "Setting up communication type: " + comm_type);

    // Initialize appropriate communication interface
    if (comm_type == "microros") {
        log_info("FourWheelDriveHardwareInterface", "on_init", "Creating micro-ROS communication interface");
        comm_ = std::make_unique<MicroRosCommunication>();
        if (!comm_->open("")) { 
            log_error("FourWheelDriveHardwareInterface", "on_init", "Failed to open micro-ROS communication");
            return hardware_interface::CallbackReturn::ERROR;
        }
    } else {
        log_error("FourWheelDriveHardwareInterface", "on_init", "Unsupported communication type: " + comm_type);
        return hardware_interface::CallbackReturn::ERROR;
    }

    log_info("FourWheelDriveHardwareInterface", "on_init", "Successfully initialized with " + comm_type + " communication");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_configure", "Starting configuration from state: " + std::string(previous_state.label()));
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelDriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelDriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "max_speed", &hw_max_speeds_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_activate", "Activating hardware interface from state: " + std::string(previous_state.label()));
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    // Set initial speed limits on activation
    if (comm_) {
        comm_->writeSpeedsToHardware(hw_max_speeds_);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_deactivate", "Deactivating hardware interface from state: " + std::string(previous_state.label()));
    if (comm_) {
        comm_->close();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!comm_ || !comm_->readStateFromHardware(hw_positions_)) {
        return hardware_interface::return_type::OK; // Keep controller running
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!comm_) {
        return hardware_interface::return_type::ERROR;
    }
    // Write both position and speed commands every cycle
    if (!comm_->writeCommandsToHardware(hw_commands_)) {
        return hardware_interface::return_type::ERROR;
    }
    // if (!comm_->writeSpeedsToHardware(hw_max_speeds_)) {
    //     return hardware_interface::return_type::ERROR;
    // }
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)
