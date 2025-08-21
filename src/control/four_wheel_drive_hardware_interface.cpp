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
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_velocity_commands_.resize(info_.joints.size(), 0.0);
    hw_max_speeds_.resize(info_.joints.size(), 10.0); // Default to 10 rad/s

    // Get communication type parameter
    std::string comm_type = info_.hardware_parameters.at("communication_type");
    log_info("FourWheelDriveHardwareInterface", "on_init", "Setting up communication type: " + comm_type);
    node_ = std::make_shared<rclcpp::Node>("shelfbot_odometry_node");
    node_spinner_ = std::thread([this]() { rclcpp::spin(node_); });

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

    odometry_ = std::make_unique<FourWheelDriveOdometry>(
        node_,
        rclcpp::Clock::SharedPtr(new rclcpp::Clock(RCL_ROS_TIME)),
        std::stod(info_.hardware_parameters.at("wheel_separation")),
        std::stod(info_.hardware_parameters.at("wheel_radius"))
    );

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
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FourWheelDriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "max_speed", &hw_max_speeds_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn FourWheelDriveHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    log_info("FourWheelDriveHardwareInterface", "on_activate", "Activating hardware interface from state: " + std::string(previous_state.label()));
    // Reset command and state vectors
    std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
    std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    
    // Set zero velocity on activation
    if (comm_) {
        comm_->writeSpeedsToHardware(hw_velocity_commands_);
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

    // Update odometry with the new wheel positions
    if (odometry_) {
        odometry_->update(hw_positions_, period);
    }

    // Log the received hardware positions for debugging
    std::stringstream ss;
    ss << "Feedback Received: [";
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
        ss << hw_positions_[i] << (i < hw_positions_.size() - 1 ? ", " : "");
    }
    ss << "]";
    log_info("FourWheelDriveHardwareInterface", "read", ss.str());

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!comm_) {
        return hardware_interface::return_type::ERROR;
    }
    // Write velocity commands every cycle
    if (!comm_->writeSpeedsToHardware(hw_velocity_commands_)) {
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)
