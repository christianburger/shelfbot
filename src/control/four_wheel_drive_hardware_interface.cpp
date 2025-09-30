#include "shelfbot/four_wheel_drive_hardware_interface.hpp"
#include "shelfbot/microros_communication.hpp"
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

    log_info("FourWheelDriveHardwareInterface", "on_init", "Initializing odometry with node clock.");
    odometry_ = std::make_unique<FourWheelDriveOdometry>(
        node_, 
        node_->get_clock(),  // Use node's clock instead of creating new one
        std::stod(info_.hardware_parameters.at("wheel_separation")), 
        std::stod(info_.hardware_parameters.at("wheel_radius")));
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
        
        std::stringstream ss;
        ss << "[hw_positions] Exporting state interface for " << info_.joints[i].name << " with initial value: " << hw_positions_[i];
        log_info("FourWheelDriveHardwareInterface", "export_state_interfaces", ss.str());
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
    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "read", 
                  "[NAV2_DIAG] Communication interface not available - CRITICAL");
        return hardware_interface::return_type::ERROR;
    }

    // Check communication health
    bool communication_healthy = comm_->is_communication_healthy();
    
    // Enhanced diagnostic logging for communication status
    static auto last_comm_status_log = time;
    if ((time - last_comm_status_log).seconds() > 3.0) {
        std::stringstream comm_ss;
        comm_ss << "[NAV2_DIAG] Communication status: " 
                << (communication_healthy ? "HEALTHY" : "UNHEALTHY")
                << " - Period: " << std::fixed << std::setprecision(3) << period.seconds() << "s";
        if (communication_healthy) {
            log_info("FourWheelDriveHardwareInterface", "read", comm_ss.str());
        } else {
            log_warn("FourWheelDriveHardwareInterface", "read", comm_ss.str());
        }
        last_comm_status_log = time;
    }

    // Attempt to read hardware state
    bool read_success = comm_->readStateFromHardware(hw_positions_);
    
    // Enhanced diagnostic logging for read operations
    static auto last_read_log = time;
    static int successful_reads = 0;
    static int failed_reads = 0;
    
    if (read_success) {
        successful_reads++;
    } else {
        failed_reads++;
    }
    
    if ((time - last_read_log).seconds() > 2.0) {
        std::stringstream read_ss;
        read_ss << std::fixed << std::setprecision(3)
                << "[NAV2_DIAG] Hardware read status: SUCCESS=" << successful_reads 
                << " FAIL=" << failed_reads 
                << " Success_rate=" << (100.0 * successful_reads / std::max(1, successful_reads + failed_reads)) << "%"
                << " Positions=[";
        for (size_t i = 0; i < hw_positions_.size(); ++i) {
            read_ss << hw_positions_[i];
            if (i < hw_positions_.size() - 1) read_ss << ", ";
        }
        read_ss << "] Comm_healthy=" << (communication_healthy ? "Yes" : "No");
        
        if (read_success && communication_healthy) {
            log_info("FourWheelDriveHardwareInterface", "read", read_ss.str());
        } else {
            log_warn("FourWheelDriveHardwareInterface", "read", read_ss.str());
        }
        
        successful_reads = 0;
        failed_reads = 0;
        last_read_log = time;
    }

    // Update odometry with enhanced diagnostics
    if (odometry_ && read_success) {
        try {
            odometry_->update(hw_positions_, period);
            
            static auto last_odom_log = time;
            if ((time - last_odom_log).seconds() > 4.0) {
                log_info("FourWheelDriveHardwareInterface", "read", 
                         "[NAV2_DIAG] Odometry updated successfully with wheel positions");
                last_odom_log = time;
            }
        } catch (const std::exception& e) {
            log_error("FourWheelDriveHardwareInterface", "read", 
                      "[NAV2_DIAG] Odometry update failed: " + std::string(e.what()));
        }
    } else if (!odometry_) {
        static auto last_odom_warn = time;
        if ((time - last_odom_warn).seconds() > 10.0) {
            log_warn("FourWheelDriveHardwareInterface", "read", 
                     "[NAV2_DIAG] Odometry not initialized - tf tree may be incomplete");
            last_odom_warn = time;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FourWheelDriveHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!comm_) {
        log_error("FourWheelDriveHardwareInterface", "write", 
                  "[NAV2_DIAG] Communication interface not available - CRITICAL");
        return hardware_interface::return_type::ERROR;
    }

    // Check communication health
    bool communication_healthy = comm_->is_communication_healthy();
    
    // Enhanced diagnostic logging for write operations
    static auto last_write_log = time;
    static int successful_writes = 0;
    static int failed_writes = 0;
    static double max_command = 0.0;
    
    // Track command magnitude for diagnostics
    double command_magnitude = 0.0;
    for (const auto& cmd : hw_velocity_commands_) {
        command_magnitude += std::abs(cmd);
    }
    max_command = std::max(max_command, command_magnitude);
    
    if ((time - last_write_log).seconds() > 1.0) {
        std::stringstream write_ss;
        write_ss << std::fixed << std::setprecision(3)
                 << "[NAV2_DIAG] Writing commands: [";
        for (size_t i = 0; i < hw_velocity_commands_.size(); ++i) {
            write_ss << hw_velocity_commands_[i];
            if (i < hw_velocity_commands_.size() - 1) write_ss << ", ";
        }
        write_ss << "] Magnitude=" << command_magnitude 
                 << " Max_seen=" << max_command
                 << " Comm_healthy=" << (communication_healthy ? "Yes" : "No")
                 << " Period=" << period.seconds() << "s";
        
        if (command_magnitude > 0.001) {
            log_info("FourWheelDriveHardwareInterface", "write", write_ss.str());
        } else {
            log_debug("FourWheelDriveHardwareInterface", "write", write_ss.str());
        }
        last_write_log = time;
    }

    // Attempt to write to hardware
    bool write_success = comm_->writeSpeedsToHardware(hw_velocity_commands_);
    
    if (write_success) {
        successful_writes++;
    } else {
        failed_writes++;
        
        if (communication_healthy) {
            log_error("FourWheelDriveHardwareInterface", "write", 
                      "[NAV2_DIAG] Write failed despite healthy communication - Hardware issue?");
            return hardware_interface::return_type::ERROR;
        } else {
            log_warn("FourWheelDriveHardwareInterface", "write", 
                     "[NAV2_DIAG] Write failed due to unhealthy communication - Expected");
            return hardware_interface::return_type::OK; // Don't treat as fatal
        }
    }
    
    // Periodic write statistics
    static auto last_stats_log = time;
    if ((time - last_stats_log).seconds() > 5.0 && (successful_writes + failed_writes) > 0) {
        double write_success_rate = 100.0 * successful_writes / (successful_writes + failed_writes);
        std::stringstream stats_ss;
        stats_ss << std::fixed << std::setprecision(1)
                 << "[NAV2_DIAG] Write statistics: SUCCESS=" << successful_writes 
                 << " FAIL=" << failed_writes 
                 << " Rate=" << write_success_rate << "%"
                 << " Max_cmd_magnitude=" << max_command;
        log_info("FourWheelDriveHardwareInterface", "write", stats_ss.str());
        
        successful_writes = 0;
        failed_writes = 0;
        max_command = 0.0;
        last_stats_log = time;
    }
    
    return hardware_interface::return_type::OK;
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(shelfbot::FourWheelDriveHardwareInterface, hardware_interface::SystemInterface)