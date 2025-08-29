#include "microros_communication.hpp"
#include "rclcpp/rclcpp.hpp"

namespace shelfbot {

MicroRosCommunication::MicroRosCommunication() : node_(nullptr) {}

MicroRosCommunication::~MicroRosCommunication() {
    close();
}

bool MicroRosCommunication::open(const std::string& /*connection_string*/) {
    try {
        node_ = std::make_shared<rclcpp::Node>("shelfbot_hardware_interface_microros_node");

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

        RCLCPP_INFO(node_->get_logger(), "Successfully opened and started spinning micro-ROS communication.");
        return true;
    } catch (const std::exception& e) {
        fprintf(stderr, "Failed to open micro-ROS communication: %s\n", e.what());
        return false;
    }
}

void MicroRosCommunication::close() {
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Closing micro-ROS communication.");
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
        RCLCPP_ERROR(node_->get_logger(), "Cannot write commands, micro-ROS communication is not active.");
        return false;
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.assign(hw_commands.begin(), hw_commands.end());
    command_publisher_->publish(msg);
    return true;
}

bool MicroRosCommunication::writeSpeedsToHardware(const std::vector<double>& hw_speeds) {
    if (!rclcpp::ok() || !speed_publisher_) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot write speeds, micro-ROS communication is not active.");
        }
        return false;
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.assign(hw_speeds.begin(), hw_speeds.end());
    speed_publisher_->publish(msg);
    return true;
}

bool MicroRosCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    if (!rclcpp::ok()) {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot read state, micro-ROS communication is not active.");
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_) {
        if (node_) {
            RCLCPP_WARN(node_->get_logger(), "No motor position data received from firmware yet.");
        }
        return false; // Return false if no data has ever been received
    }

    // Ensure the destination vector has the same size to avoid out-of-bounds access
    if (hw_positions.size() != latest_hw_positions_.size()) {
        hw_positions.resize(latest_hw_positions_.size());
    }

    hw_positions = latest_hw_positions_;
    
    // Log the read positions for debugging
    std::stringstream ss;
    ss << "Reading positions: [";
    for (size_t i = 0; i < hw_positions.size(); ++i) {
        ss << hw_positions[i] << (i < hw_positions.size() - 1 ? ", " : "");
    }
    ss << "]";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    return true;
}

void MicroRosCommunication::position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_hw_positions_.assign(msg->data.begin(), msg->data.end());
    state_received_ = true;

    // Log the received positions for debugging
    std::stringstream ss;
    ss << "Received new motor positions: [";
    for (size_t i = 0; i < latest_hw_positions_.size(); ++i) {
        ss << latest_hw_positions_[i] << (i < latest_hw_positions_.size() - 1 ? ", " : "");
    }
    ss << "]";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
}

}
