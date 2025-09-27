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
        log_error("MicroRosCommunication", "writeSpeedsToHardware", "Cannot write speeds, micro-ROS communication is not active.");
        return false;
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.assign(hw_speeds.begin(), hw_speeds.end());
    speed_publisher_->publish(msg);
    return true;
}

bool MicroRosCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    if (!rclcpp::ok()) {
        log_error("MicroRosCommunication", "readStateFromHardware", "Cannot read state, micro-ROS communication is not active.");
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_) {
        log_warn("MicroRosCommunication", "readStateFromHardware", "No motor position data received from firmware yet.");
        return false;
    }

    size_t size_to_copy = std::min(hw_positions.size(), latest_hw_positions_.size());
    for (size_t i = 0; i < size_to_copy; ++i) {
        hw_positions[i] = latest_hw_positions_[i];
    }
    
    std::stringstream ss;
    ss << "[hw_positions] Reading positions: [";
    for (size_t i = 0; i < hw_positions.size(); ++i) {
        ss << hw_positions[i] << (i < hw_positions.size() - 1 ? ", " : "");
    }
    ss << "]";
    log_info("MicroRosCommunication", "readStateFromHardware", ss.str());

    return true;
}

void MicroRosCommunication::position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!node_) {
        log_error("MicroRosCommunication", "position_callback", "Node not initialized");
        return;
    }
    
    auto now = node_->now();
    
    // FIX: Safe time subtraction with validation
    double time_since_last_msg = 0.0;
    if (last_received_time_.nanoseconds() > 0) {
        try {
            time_since_last_msg = (now - last_received_time_).seconds();
        } catch (const std::runtime_error& e) {
            log_error("MicroRosCommunication", "position_callback", 
                     std::string("Time subtraction error: ") + e.what());
            // Reset to avoid repeated errors
            last_received_time_ = now;
            time_since_last_msg = 0.0;
        }
    }
    
    if (last_received_time_.nanoseconds() > 0 && time_since_last_msg > 1.0) {
        log_warn("MicroRosCommunication", "position_callback", 
                "Long interval since last motor data: " + std::to_string(time_since_last_msg) + " seconds");
    }
    
    last_received_time_ = now;
    
    latest_hw_positions_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        latest_hw_positions_[i] = static_cast<double>(msg->data[i]);
    }
    
    state_received_ = true;

    std::stringstream ss;
    ss << "[hw_positions] Received new motor positions: [";
    for (size_t i = 0; i < latest_hw_positions_.size(); ++i) {
        ss << latest_hw_positions_[i] << (i < latest_hw_positions_.size() - 1 ? ", " : "");
    }
    ss << "] - Time since last message: " << time_since_last_msg << "s";
    log_info("MicroRosCommunication", "position_callback", ss.str());
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
