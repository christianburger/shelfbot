#include "shelfbot/microros_communication.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shelfbot/shelfbot_utils.hpp"

namespace shelfbot {

MicroRosCommunication::MicroRosCommunication() : node_(nullptr) {
    // FIX: Don't initialize last_received_time_ here - wait until node is created
    // The clock type will be determined by the node's clock
}

MicroRosCommunication::~MicroRosCommunication() {
    close();
}

bool MicroRosCommunication::open(const std::string& /*connection_string*/) {
    try {
        node_ = std::make_shared<rclcpp::Node>("shelfbot_hardware_interface_microros_node");

        // FIX: Initialize last_received_time_ with the node's clock type AFTER node is created
        last_received_time_ = node_->now();
        
        command_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/motor_command", 10);

        speed_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/set_speed", 10);

        // Subscriber for motor position feedback
        position_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/motor_positions", rclcpp::SensorDataQoS(),
            std::bind(&MicroRosCommunication::position_callback, this, std::placeholders::_1));

        executor_.add_node(node_);
        executor_thread_ = std::thread([this]() { this->executor_.spin(); });

        log_info("MicroRosCommunication", "open", "Successfully opened and started spinning micro-ROS communication.");
        return true;
    } catch (const std::exception& e) {
        log_error("MicroRosCommunication", "open", std::string("Failed to open micro-ROS communication: ") + e.what());
        return false;
    }
}

void MicroRosCommunication::close() {
    if (node_) {
        log_info("MicroRosCommunication", "close", "Closing micro-ROS communication.");
        executor_.cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        command_publisher_.reset();
        speed_publisher_.reset();
        position_subscriber_.reset();
        node_.reset();
    }
}

bool MicroRosCommunication::writeCommandsToHardware(const std::vector<double>& hw_commands) {
    if (!rclcpp::ok() || !command_publisher_) {
        log_error("MicroRosCommunication", "writeCommandsToHardware", "Cannot write commands, micro-ROS communication is not active.");
        return false;
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.assign(hw_commands.begin(), hw_commands.end());
    command_publisher_->publish(msg);
    return true;
}

bool MicroRosCommunication::writeSpeedsToHardware(const std::vector<double>& hw_speeds) {
    if (!rclcpp::ok() || !speed_publisher_) {
        log_error("MicroRosCommunication", "writeSpeedsToHardware", 
                  "[NAV2_DIAG] Cannot write speeds, micro-ROS communication is not active - CRITICAL");
        return false;
    }

    // Enhanced diagnostic logging for command transmission
    static auto last_write_log = node_->now();
    static int write_count = 0;
    static double max_speed_seen = 0.0;
    static double total_commands_sent = 0.0;
    
    write_count++;
    double command_magnitude = 0.0;
    for (const auto& speed : hw_speeds) {
        command_magnitude += std::abs(speed);
    }
    max_speed_seen = std::max(max_speed_seen, command_magnitude);
    total_commands_sent += command_magnitude;
    
    // Publish the command
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.assign(hw_speeds.begin(), hw_speeds.end());
    speed_publisher_->publish(msg);
    
    // Detailed logging every 2 seconds
    if ((node_->now() - last_write_log).seconds() > 2.0) {
        std::stringstream write_ss;
        write_ss << std::fixed << std::setprecision(3)
                 << "[NAV2_DIAG] MicroROS write status: count=" << write_count
                 << " rate=" << (write_count / 2.0) << "Hz"
                 << " current_magnitude=" << command_magnitude
                 << " max_seen=" << max_speed_seen
                 << " avg_magnitude=" << (write_count > 0 ? total_commands_sent / write_count : 0.0)
                 << " speeds=[";
        for (size_t i = 0; i < hw_speeds.size(); ++i) {
            write_ss << hw_speeds[i];
            if (i < hw_speeds.size() - 1) write_ss << ", ";
        }
        write_ss << "]";
        
        if (command_magnitude > 0.001) {
            log_info("MicroRosCommunication", "writeSpeedsToHardware", write_ss.str());
        } else {
            log_debug("MicroRosCommunication", "writeSpeedsToHardware", write_ss.str());
        }
        
        write_count = 0;
        max_speed_seen = 0.0;
        total_commands_sent = 0.0;
        last_write_log = node_->now();
    }
    
    return true;
}

bool MicroRosCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    if (!rclcpp::ok()) {
        log_error("MicroRosCommunication", "readStateFromHardware", 
                  "[NAV2_DIAG] Cannot read state, micro-ROS communication is not active - CRITICAL");
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Enhanced diagnostic for state reception
    static auto last_read_log = node_->now();
    static int successful_reads = 0;
    static int failed_reads = 0;
    static auto last_data_time = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    
    bool read_success = state_received_;
    
    if (!read_success) {
        failed_reads++;
        
        static auto last_fail_log = node_->now();
        if ((node_->now() - last_fail_log).seconds() > 5.0) {
            log_warn("MicroRosCommunication", "readStateFromHardware", 
                     "[NAV2_DIAG] No motor position data received from firmware - Check ESP32 connection");
            last_fail_log = node_->now();
        }
        return false;
    }
    
    successful_reads++;
    last_data_time = last_received_time_;
    
    // Copy data
    size_t size_to_copy = std::min(hw_positions.size(), latest_hw_positions_.size());
    for (size_t i = 0; i < size_to_copy; ++i) {
        hw_positions[i] = latest_hw_positions_[i];
    }
    
    // Periodic detailed logging
    if ((node_->now() - last_read_log).seconds() > 2.0) {
        double data_age = (node_->now() - last_data_time).seconds();
        double success_rate = successful_reads * 100.0 / std::max(1, successful_reads + failed_reads);
        
        std::stringstream read_ss;
        read_ss << std::fixed << std::setprecision(3)
                << "[NAV2_DIAG] MicroROS read status: SUCCESS=" << successful_reads
                << " FAIL=" << failed_reads 
                << " rate=" << success_rate << "%"
                << " data_age=" << data_age << "s"
                << " positions=[";
        for (size_t i = 0; i < hw_positions.size(); ++i) {
            read_ss << hw_positions[i];
            if (i < hw_positions.size() - 1) read_ss << ", ";
        }
        read_ss << "] comm_healthy=" << (is_communication_healthy() ? "Yes" : "No");
        
        if (success_rate > 80.0 && data_age < 1.0) {
            log_info("MicroRosCommunication", "readStateFromHardware", read_ss.str());
        } else {
            log_warn("MicroRosCommunication", "readStateFromHardware", read_ss.str());
        }
        
        successful_reads = 0;
        failed_reads = 0;
        last_read_log = node_->now();
    }

    return true;
}

void MicroRosCommunication::position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!node_) {
        log_error("MicroRosCommunication", "position_callback", 
                  "[NAV2_DIAG] Node not initialized in position callback - CRITICAL");
        return;
    }
    
    auto now = node_->now();
    
    // Enhanced callback diagnostics
    static auto last_callback_log = now;
    static int callback_count = 0;
    static double min_interval = std::numeric_limits<double>::max();
    static double max_interval = 0.0;
    static auto first_callback_time = now;
    
    callback_count++;
    
    // Calculate interval since last message
    double interval = 0.0;
    if (last_received_time_.nanoseconds() > 0) {
        try {
            interval = (now - last_received_time_).seconds();
            min_interval = std::min(min_interval, interval);
            max_interval = std::max(max_interval, interval);
        } catch (const std::runtime_error& e) {
            log_error("MicroRosCommunication", "position_callback", 
                     "[NAV2_DIAG] Time calculation error: " + std::string(e.what()));
            last_received_time_ = now;
            interval = 0.0;
        }
    }
    
    last_received_time_ = now;
    
    // Update position data
    latest_hw_positions_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        latest_hw_positions_[i] = static_cast<double>(msg->data[i]);
    }
    
    state_received_ = true;
    
    // Periodic detailed callback statistics
    if ((now - last_callback_log).seconds() > 3.0) {
        double total_runtime = (now - first_callback_time).seconds();
        double avg_rate = callback_count / std::max(0.001, total_runtime);
        
        std::stringstream cb_ss;
        cb_ss << std::fixed << std::setprecision(3)
              << "[NAV2_DIAG] Motor data callback stats: count=" << callback_count
              << " rate=" << avg_rate << "Hz"
              << " interval: min=" << min_interval << "s max=" << max_interval << "s current=" << interval << "s"
              << " positions=[";
        for (size_t i = 0; i < std::min<size_t>(latest_hw_positions_.size(), 4); ++i) {
            cb_ss << latest_hw_positions_[i];
            if (i < std::min<size_t>(latest_hw_positions_.size(), 4) - 1) cb_ss << ", ";
        }
        if (latest_hw_positions_.size() > 4) cb_ss << "...";
        cb_ss << "]";
        
        if (avg_rate > 5.0 && interval < 1.0) {
            log_info("MicroRosCommunication", "position_callback", cb_ss.str());
        } else {
            log_warn("MicroRosCommunication", "position_callback", 
                    cb_ss.str() + " - LOW RATE OR HIGH LATENCY");
        }
        
        // Reset counters for next period
        callback_count = 0;
        min_interval = std::numeric_limits<double>::max();
        max_interval = 0.0;
        last_callback_log = now;
        first_callback_time = now;
    }
    
    // Log data reception issues
    if (interval > 2.0) {
        log_warn("MicroRosCommunication", "position_callback", 
                "[NAV2_DIAG] Large gap in motor data: " + std::to_string(interval) + "s - Check ESP32");
    }
}

bool MicroRosCommunication::is_communication_healthy() const {
    if (!node_) return false;
    
    auto now = node_->now();
    
    // FIX: Safe time comparison with validation
    if (last_received_time_.nanoseconds() == 0) {
        return false; // No messages received yet
    }
    
    try {
        auto time_since_last_msg = (now - last_received_time_).seconds();
        bool healthy = time_since_last_msg < max_allowed_interval_.seconds();
        
        if (!healthy) {
            log_warn("MicroRosCommunication", "is_communication_healthy", 
                    "Communication unhealthy - last message received " + 
                    std::to_string(time_since_last_msg) + " seconds ago");
        }
        
        return healthy;
    } catch (const std::runtime_error& e) {
        log_error("MicroRosCommunication", "is_communication_healthy", 
                 std::string("Time comparison error: ") + e.what());
        return false;
    }
}

}
