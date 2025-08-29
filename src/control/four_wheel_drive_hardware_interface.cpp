#include "four_wheel_drive_hardware_interface.hpp"
#include "microros_communication.hpp"
#include <sstream>

namespace shelfbot {

FourWheelDriveHardwareInterface::FourWheelDriveHardwareInterface() {
    log_info("FourWheelDriveHardwareInterface", "Constructor", "Hardware interface constructor called.");
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    log_info("FourWheelDriveHardwareInterface", "on_init", "--- Starting on_init ---");
    
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        log_error("FourWheelDriveHardwareInterface", "on_init", "Base class on_init failed.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    log_info("FourWheelDriveHardwareInterface", "on_init", "Initializing storage vectors.");
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_velocity_commands_.resize(info_.joints.size(), 0.0);
    hw_max_speeds_.resize(info_.joints.size(), 10.0);
    log_info("FourWheelDriveHardwareInterface", "on_init", "Storage vectors initialized for " + std::to_string(info_.joints.size()) + " joints.");

    log_info("FourWheelDriveHardwareInterface", "on_init", "Reading hardware parameters.");
    std::string comm_type = info_.hardware_parameters.at("communication_type");
    log_info("FourWheelDriveHardwareInterface", "on_init", "Communication type parameter found: " + comm_type);

    log_info("FourWheelDriveHardwareInterface", "on_init", "Real robot mode selected. Initializing ROS node for odometry.");
    node_ = std::make_shared<rclcpp::Node>("shelfbot_odometry_node");
    node_spinner_ = std::thread([this]() { rclcpp::spin(node_); });
    log_info("FourWheelDriveHardwareInterface", "on_init", "ROS node and spinner created.");

    if (comm_type == "microros") {
        log_info("FourWheelDriveHardwareInterface", "on_init", "Creating micro-ROS communication interface.");
        comm_ = std::make_unique<MicroRosCommunication>();
        if (!comm_->open("")) { 
            log_error("FourWheelDriveHardwareInterface", "on_init", "Failed to open micro-ROS communication.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        log_info("FourWheelDriveHardwareInterface", "on_init", "micro-ROS communication opened successfully.");
    } else {
        log_error("FourWheelDriveHardwareInterface", "on_init", "Unsupported communication type: " + comm_type);
        return hardware_interface::CallbackReturn::ERROR;
    }

    log_info("FourWheelDriveHardwareInterface", "on_init", "Initializing odometry.");
    odometry_ = std::make_unique<FourWheelDriveOdometry>(
        node_,
        rclcpp::Clock::SharedPtr(new rclcpp::Clock(RCL_ROS_TIME)),
        std::stod(info_.hardware_parameters.at("wheel_separation")),
        std::stod(info_.hardware_parameters.at("wheel_radius"))
    );
    log_info("FourWheelDriveHardwareInterface", "on_init", "Odometry initialized successfully.");

    log_info("FourWheelDriveHardwareInterface", "on_init", "--- on_init successful (Real Robot) ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_configure", "--- Starting on_configure from state: " + std::string(previous_state.label()) + " ---");
    // Nothing to configure here, just logging
    log_info("FourWheelDriveHardwareInterface", "on_configure", "--- on_configure successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FourWheelDriveHardwareInterface::export_state_interfaces() {
    log_info("FourWheelDriveHardwareInterface", "export_state_interfaces", "--- Exporting state interfaces ---");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
        log_info("FourWheelDriveHardwareInterface", "export_state_interfaces", "Exported state interfaces for joint: " + info_.joints[i].name);
    }
    log_info("FourWheelDriveHardwareInterface", "export_state_interfaces", "--- State interfaces exported successfully ---");
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelDriveHardwareInterface::export_command_interfaces() {
    log_info("FourWheelDriveHardwareInterface", "export_command_interfaces", "--- Exporting command interfaces ---");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "max_speed", &hw_max_speeds_[i]);
        log_info("FourWheelDriveHardwareInterface", "export_command_interfaces", "Exported command interfaces for joint: " + info_.joints[i].name);
    }
    log_info("FourWheelDriveHardwareInterface", "export_command_interfaces", "--- Command interfaces exported successfully ---");
    return command_interfaces;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_activate", "--- Starting on_activate from state: " + std::string(previous_state.label()) + " ---");
    log_info("FourWheelDriveHardwareInterface", "on_activate", "Resetting command and state vectors to zero.");
    std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    
    if (comm_) {
        log_info("FourWheelDriveHardwareInterface", "on_activate", "Writing zero velocity to hardware on activation.");
        comm_->writeSpeedsToHardware(hw_velocity_commands_);
    }
    log_info("FourWheelDriveHardwareInterface", "on_activate", "--- on_activate successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_deactivate", "--- Starting on_deactivate from state: " + std::string(previous_state.label()) + " ---");
    if (comm_) {
        log_info("FourWheelDriveHardwareInterface", "on_deactivate", "Closing hardware communication.");
        comm_->close();
    }
    log_info("FourWheelDriveHardwareInterface", "on_deactivate", "--- on_deactivate successful ---");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!comm_ || !comm_->readStateFromHardware(hw_positions_)) {
        log_warn("FourWheelDriveHardwareInterface", "read", "Failed to read from hardware or communication not available.");
        return hardware_interface::return_type::OK; // Keep controller running
    }

    if (odometry_) {
        odometry_->update(hw_positions_, period);
    }

    std::stringstream ss;
    ss << "Read from hardware: Positions = [";
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
        ss << hw_positions_[i] << (i < hw_positions_.size() - 1 ? ", " : "");
    }
    ss << "]";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "write", "Communication interface not available.");
        return hardware_interface::return_type::ERROR;
    }

    std::stringstream ss;
    ss << "Writing to hardware: Commands = [";
    for (size_t i = 0; i < hw_velocity_commands_.size(); ++i) {
        ss << hw_velocity_commands_[i] << (i < hw_velocity_commands_.size() - 1 ? ", " : "");
    }
    ss << "]";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    if (!comm_->writeSpeedsToHardware(hw_velocity_commands_)) {
        log_error("FourWheelDriveHardwareInterface", "write", "Failed to write speeds to hardware.");
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)