#pragma once

#include "shelfbot/communication_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <mutex>
#include <thread>

namespace shelfbot {

class MicroRosCommunication : public CommunicationInterface {
public:
    MicroRosCommunication();
    ~MicroRosCommunication() override;

    bool open(const std::string& connection_string) override;
    void close() override;
    bool writeCommandsToHardware(const std::vector<double>& hw_commands) override;
    bool writeSpeedsToHardware(const std::vector<double>& hw_speeds) override;
    bool readStateFromHardware(std::vector<double>& hw_positions) override;
    bool is_communication_healthy() const override;  // ADD override KEYWORD

private:
    void position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_subscriber_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread executor_thread_;

    std::mutex state_mutex_;
    std::vector<double> latest_hw_positions_;
    bool state_received_ = false;

    rclcpp::Time last_received_time_;
    rclcpp::Duration max_allowed_interval_{1, 0}; // 1 second
};

}
