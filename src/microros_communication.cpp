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

        // Publisher can use default reliable QoS
        command_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/motor_command", 10);

        // Subscriber needs a QoS profile that matches the micro-ROS agent's publisher
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        state_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/motor_state", qos,
            std::bind(&MicroRosCommunication::state_callback, this, std::placeholders::_1));

        // Create and spin the executor in a separate thread
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
        state_subscriber_.reset();
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

bool MicroRosCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot read state, micro-ROS communication is not active.");
        return false;
    }

    if (!state_received_) {
        return false; // Not an error, just no data yet.
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    hw_positions = latest_hw_positions_;
    return true;
}

void MicroRosCommunication::state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_hw_positions_.assign(msg->data.begin(), msg->data.end());
    state_received_ = true;
}

}
