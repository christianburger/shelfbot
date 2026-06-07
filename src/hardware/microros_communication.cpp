#include "shelfbot/microros_communication.hpp"
#include "shelfbot/shelfbot_utils.hpp"  // pulls in log_zip.hpp

namespace shelfbot {

MicroRosCommunication::MicroRosCommunication() : node_(nullptr) {}

MicroRosCommunication::~MicroRosCommunication() { close(); }

bool MicroRosCommunication::open(const std::string& /*connection_string*/) {
    try {
        node_ = std::make_shared<rclcpp::Node>(
            "shelfbot_hardware_interface_microros_node");
        last_received_time_ = node_->now();

        command_publisher_  = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/motor_command", 10);
        speed_publisher_    = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/set_speed", 10);
        position_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/shelfbot_firmware/motor_positions", rclcpp::SensorDataQoS(),
            std::bind(&MicroRosCommunication::position_callback, this,
                      std::placeholders::_1));

        executor_.add_node(node_);
        executor_thread_ = std::thread([this]() { executor_.spin(); });

        log_zip_s("UROS", "OPEN", {{"st", "ok"}});
        log_info("MicroRosCommunication", "open",
                 "Successfully opened and started spinning micro-ROS communication.");
        return true;
    } catch (const std::exception& e) {
        log_zip_s("UROS", "OPEN", {{"st", "err"}});
        log_error("MicroRosCommunication", "open",
                  std::string("Failed to open micro-ROS communication: ") + e.what());
        return false;
    }
}

void MicroRosCommunication::close() {
    if (node_) {
        log_zip_s("UROS", "CLOSE", {{"st", "ok"}});
        log_info("MicroRosCommunication", "close", "Closing micro-ROS communication.");
        executor_.cancel();
        if (executor_thread_.joinable()) executor_thread_.join();
        command_publisher_.reset();
        speed_publisher_.reset();
        position_subscriber_.reset();
        node_.reset();
    }
}

bool MicroRosCommunication::writeCommandsToHardware(const std::vector<double>& hw_commands) {
    if (!rclcpp::ok() || !command_publisher_) {
        log_error("MicroRosCommunication", "writeCommandsToHardware",
                  "Cannot write commands, micro-ROS communication is not active.");
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
                  "Cannot write speeds, micro-ROS communication is not active.");
        return false;
    }
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.assign(hw_speeds.begin(), hw_speeds.end());
    speed_publisher_->publish(msg);
    return true;
}

bool MicroRosCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    if (!rclcpp::ok()) {
        log_error("MicroRosCommunication", "readStateFromHardware",
                  "Cannot read state, micro-ROS communication is not active.");
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_) {
        // Not a log_zip site — fires every cycle before first msg; too noisy.
        log_warn("MicroRosCommunication", "readStateFromHardware",
                 "No motor position data received from firmware yet.");
        return false;
    }

    size_t n = std::min(hw_positions.size(), latest_hw_positions_.size());
    for (size_t i = 0; i < n; ++i)
        hw_positions[i] = latest_hw_positions_[i];

    return true;
}

void MicroRosCommunication::position_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!node_) {
        log_error("MicroRosCommunication", "position_callback", "Node not initialized");
        return;
    }

    auto   now                = node_->now();
    double time_since_last_ms = 0.0;

    if (last_received_time_.nanoseconds() > 0) {
        try {
            time_since_last_ms = (now - last_received_time_).seconds() * 1000.0;
        } catch (const std::runtime_error& e) {
            log_error("MicroRosCommunication", "position_callback",
                     std::string("Time subtraction error: ") + e.what());
            last_received_time_ = now;
        }
    }

    last_received_time_ = now;

    latest_hw_positions_.resize(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i)
        latest_hw_positions_[i] = static_cast<double>(msg->data[i]);

    bool was_first = !state_received_;
    state_received_ = true;

    // ── log_zip: new positions from firmware ─────────────────────────────
    // Include inter-message gap in ms to catch timing issues
    log_zip("UROS", "POS", {
        {"p0",   latest_hw_positions_.size() > 0 ? latest_hw_positions_[0] : 0.0},
        {"p1",   latest_hw_positions_.size() > 1 ? latest_hw_positions_[1] : 0.0},
        {"p2",   latest_hw_positions_.size() > 2 ? latest_hw_positions_[2] : 0.0},
        {"p3",   latest_hw_positions_.size() > 3 ? latest_hw_positions_[3] : 0.0},
        {"dt_ms", time_since_last_ms},
        {"first", was_first ? 1.0 : 0.0}
    });

    if (time_since_last_ms > 1000.0) {
        log_zip("UROS", "GAP", {{"dt_ms", time_since_last_ms}});
        log_warn("MicroRosCommunication", "position_callback",
                "Long interval since last motor data: " +
                std::to_string(time_since_last_ms / 1000.0) + " seconds");
    }
}

bool MicroRosCommunication::is_communication_healthy() const {
    if (!node_) return false;
    if (last_received_time_.nanoseconds() == 0) return false;

    try {
        auto   now   = node_->now();
        double age_s = (now - last_received_time_).seconds();
        bool   ok    = age_s < max_allowed_interval_.seconds();

        if (!ok) {
            // ── log_zip: communication health failure ─────────────────────
            log_zip("UROS", "HLTH", {{"ok", 0}, {"age_s", age_s}});
            log_warn("MicroRosCommunication", "is_communication_healthy",
                    "Communication unhealthy - last message received " +
                    std::to_string(age_s) + " seconds ago");
        }
        return ok;
    } catch (const std::runtime_error& e) {
        log_error("MicroRosCommunication", "is_communication_healthy",
                 std::string("Time comparison error: ") + e.what());
        return false;
    }
}

} // namespace shelfbot
