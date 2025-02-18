#include "four_wheel_drive_hardware_interface.hpp"

namespace shelfbot {

FourWheelDriveHardwareInterface::FourWheelDriveHardwareInterface() {
    log_info("FourWheelDriveHardwareInterface", "Constructor", "Starting hardware interface initialization");
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    log_info("FourWheelDriveHardwareInterface", "on_init", "Starting initialization sequence");
    
    // Log all hardware parameters
    log_info("FourWheelDriveHardwareInterface", "on_init", "Hardware Parameters:");
    for (const auto& param : info.hardware_parameters) {
        log_info("FourWheelDriveHardwareInterface", "on_init", "Parameter: " + param.first + " = " + param.second);
    }

    if (hardware_interface::SystemInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS) {
        log_error("FourWheelDriveHardwareInterface", "on_init", "Base initialization failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    // Get communication type parameter
    std::string comm_type = info_.hardware_parameters.at("communication_type");
    log_info("FourWheelDriveHardwareInterface", "on_init", 
             "Setting up communication type: " + comm_type);

    // Initialize appropriate communication interface
    if (comm_type == "rest") {
        if (!info_.hardware_parameters.count("base_url")) {
            log_error("FourWheelDriveHardwareInterface", "on_init", 
                     "REST communication requires base_url parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::string base_url = info_.hardware_parameters.at("base_url");
        log_info("FourWheelDriveHardwareInterface", "on_init", 
                 "Creating REST communication with URL: " + base_url);
        comm_ = std::make_unique<RestCommunication>();
        if (!comm_->open(base_url)) {
            log_error("FourWheelDriveHardwareInterface", "on_init", 
                     "Failed to open REST communication");
            return hardware_interface::CallbackReturn::ERROR;
        }
    } else if (comm_type == "mock") {
        log_info("FourWheelDriveHardwareInterface", "on_init", 
                 "Creating Mock communication interface");
        comm_ = std::make_unique<MockCommunication>();
        if (!comm_->open("mock")) {
            log_error("FourWheelDriveHardwareInterface", "on_init", 
                     "Failed to open mock communication");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    log_info("FourWheelDriveHardwareInterface", "on_init", 
             "Successfully initialized with " + comm_type + " communication");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_configure", "Starting configuration from state: " + std::string(previous_state.label()));
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelDriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    log_info("FourWheelDriveHardwareInterface", "export_state_interfaces", "Exporting state interfaces for " + std::to_string(info_.joints.size()) + " joints");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        log_info("FourWheelDriveHardwareInterface", "export_state_interfaces",
                 "Creating state interface for joint: " + info_.joints[i].name);
        state_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[i]
        );
    }

    log_info("FourWheelDriveHardwareInterface", "export_state_interfaces",
             "Successfully exported " + std::to_string(state_interfaces.size()) + " state interfaces");
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelDriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    log_info("FourWheelDriveHardwareInterface", "export_command_interfaces",
             "Exporting command interfaces for " + std::to_string(info_.joints.size()) + " joints");

    for (size_t i = 0; i < info_.joints.size(); i++) {
        log_info("FourWheelDriveHardwareInterface", "export_command_interfaces",
                 "Creating command interface for joint: " + info_.joints[i].name);
        command_interfaces.emplace_back(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]
        );
    }

    log_info("FourWheelDriveHardwareInterface", "export_command_interfaces",
             "Successfully exported " + std::to_string(command_interfaces.size()) + " command interfaces");
    return command_interfaces;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_activate", 
             "Activating hardware interface from state: " + std::string(previous_state.label()));

    // Reset commands/positions
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    
    // Explicitly register interfaces
    for (size_t i = 0; i < info_.joints.size(); i++) {
        log_info("FourWheelDriveHardwareInterface", "on_activate", 
                 "Activating joint: " + info_.joints[i].name);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_deactivate", "Deactivating hardware interface from state: " + std::string(previous_state.label()));
    if (comm_) {
        comm_->close();
        log_info("FourWheelDriveHardwareInterface", "on_deactivate", "Communication interface closed");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    
    log_debug("FourWheelDriveHardwareInterface", "read", "Reading hardware state");

    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "read", "Communication interface not initialized");
        return hardware_interface::return_type::OK;  // Changed from ERROR to OK
    }

    // Even if read fails, return OK to keep the controller running
    if (!comm_->readStateFromHardware(hw_positions_)) {
        log_error("FourWheelDriveHardwareInterface", "read", "Failed to read state - using last known positions");
        // Keep using last known positions
        return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    
    log_debug("FourWheelDriveHardwareInterface", "write", "Writing at time: " + std::to_string(time.seconds()) + ", period: " + std::to_string(period.seconds()));

    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "write", "Communication interface not initialized");
        return hardware_interface::return_type::ERROR;
    }

    for (size_t i = 0; i < hw_commands_.size(); ++i) {
        log_debug("FourWheelDriveHardwareInterface", "write", 
                  "Joint " + std::to_string(i) + " command: " + std::to_string(hw_commands_[i]));
    }

    if (!comm_->writeCommandsToHardware(hw_commands_)) {
        log_error("FourWheelDriveHardwareInterface", "write", "Failed to write commands to hardware");
        return hardware_interface::return_type::ERROR;
    }

    log_debug("FourWheelDriveHardwareInterface", "write", "Successfully wrote commands to hardware");
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)