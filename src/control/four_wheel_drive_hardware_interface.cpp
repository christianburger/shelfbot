#include "four_wheel_drive_hardware_interface.hpp"

namespace shelfbot {

FourWheelDriveHardwareInterface::FourWheelDriveHardwareInterface() {
    log_info("FourWheelDriveHardwareInterface", "Constructor", "Starting hardware interface initialization");
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    log_info("FourWheelDriveHardwareInterface", "on_init", "Starting hardware interface initialization");
    
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        log_error("FourWheelDriveHardwareInterface", "on_init", "Failed to initialize base class");
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    log_info("FourWheelDriveHardwareInterface", "on_init", "Vectors resized to " + std::to_string(info_.joints.size()) + " elements");

    for (const auto& joint : info_.joints) {
        log_info("FourWheelDriveHardwareInterface", "on_init", "Found joint: " + joint.name);
    }

    if (info_.hardware_parameters.count("communication_type") > 0 && 
        info_.hardware_parameters.count("base_url") > 0) {
        
        std::string comm_type = info_.hardware_parameters.at("communication_type");
        std::string base_url = info_.hardware_parameters.at("base_url");
        
        log_info("FourWheelDriveHardwareInterface", "on_init", "Communication type: " + comm_type + ", Base URL: " + base_url);

        if (comm_type == "rest") {
            log_info("FourWheelDriveHardwareInterface", "on_init", "Initializing REST communication");
            comm_ = std::unique_ptr<CommunicationInterface>(new RestCommunication());
            if (!comm_->open(base_url)) {
                log_error("FourWheelDriveHardwareInterface", "on_init", "Failed to open REST communication");
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            log_info("FourWheelDriveHardwareInterface", "on_init", "Initializing Mock communication");
            comm_ = std::unique_ptr<CommunicationInterface>(new MockCommunication());
            if (!comm_->open("/dev/ttyUSB0")) {
                log_error("FourWheelDriveHardwareInterface", "on_init", "Failed to open mock communication");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    log_info("FourWheelDriveHardwareInterface", "on_init", "Hardware interface initialized successfully");
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

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_activate", "Activating hardware interface from state: " + std::string(previous_state.label()));
    
    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "on_activate", "Communication interface not initialized");
        return hardware_interface::CallbackReturn::ERROR;
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

hardware_interface::return_type FourWheelDriveHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    log_debug("FourWheelDriveHardwareInterface", "read", "Reading hardware state at time: " + std::to_string(time.seconds()));
    
    if (!comm_->readStateFromHardware(hw_positions_)) {
        log_error("FourWheelDriveHardwareInterface", "read", "Failed to read hardware state");
        return hardware_interface::return_type::ERROR;
    }
    
    for (size_t i = 0; i < hw_positions_.size(); i++) {
        log_debug("FourWheelDriveHardwareInterface", "read", 
                  "Joint " + info_.joints[i].name + " position: " + std::to_string(hw_positions_[i]));
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    log_debug("FourWheelDriveHardwareInterface", "write", "Writing commands at time: " + std::to_string(time.seconds()));
    
    for (size_t i = 0; i < hw_commands_.size(); i++) {
        log_debug("FourWheelDriveHardwareInterface", "write", 
                  "Joint " + info_.joints[i].name + " command: " + std::to_string(hw_commands_[i]));
    }
    
    if (!comm_->writeCommandsToHardware(hw_commands_)) {
        log_error("FourWheelDriveHardwareInterface", "write", "Failed to write commands to hardware");
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)