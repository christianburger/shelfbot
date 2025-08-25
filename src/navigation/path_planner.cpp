#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"  // Needed for state and diagnostic messages
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For TF conversions
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/transform_stamped.hpp"

// Define the states for our mission
enum class MissionState {
    INITIALIZING,
    SEARCHING_FOR_DESTINATION,
    RETURNING_TO_ORIGIN,
    CONFIRMING_ORIGIN,
    MISSION_COMPLETE
};

// Utility function to convert MissionState enum to string
std::string missionStateToString(MissionState state) {
    switch (state) {
        case MissionState::INITIALIZING: return "INITIALIZING";
        case MissionState::SEARCHING_FOR_DESTINATION: return "SEARCHING_FOR_DESTINATION";
        case MissionState::RETURNING_TO_ORIGIN: return "RETURNING_TO_ORIGIN";
        case MissionState::CONFIRMING_ORIGIN: return "CONFIRMING_ORIGIN";
        case MissionState::MISSION_COMPLETE: return "MISSION_COMPLETE";
        default: return "UNKNOWN";
    }
}

class MissionControlNode : public rclcpp::Node
{
public:
    MissionControlNode() : Node("mission_control_node")
    {
        // Publisher for robot velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/four_wheel_drive_controller/cmd_vel", 10);

        // --- NEW: Publishers for state and diagnostics ---
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/mission_control/state", 10);
        diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>("/mission_control/diagnostics", 10);

        // Subscriber to the AprilTag poses
        tag_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/tag_poses", 10, std::bind(&MissionControlNode::tagPosesCallback, this, std::placeholders::_1));

        // Initialize TF2 listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer to run the main state machine loop at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionControlNode::stateMachineLoop, this));

        RCLCPP_INFO(this->get_logger(), "Mission Control Node has been initialized.");
        RCLCPP_INFO(this->get_logger(), "Current state: %s", missionStateToString(current_state_).c_str());
    }

private:
    // Callback for the /tag_poses subscriber
    void tagPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!msg->poses.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Received %zu tag poses.", msg->poses.size());
        }
    }

    // Main state machine loop
    void stateMachineLoop()
    {
        // --- NEW: Publish state and diagnostic messages ---
        publishState();
        publishDiagnostics();

        // This function will contain the logic for each state.
        switch (current_state_)
        {
            case MissionState::INITIALIZING:
                // TODO: Add logic to find the origin tag
                break;
            case MissionState::SEARCHING_FOR_DESTINATION:
                // TODO: Add logic to search for the destination tag
                break;
            case MissionState::RETURNING_TO_ORIGIN:
                // TODO: Add logic to return to the origin
                break;
            case MissionState::CONFIRMING_ORIGIN:
                // TODO: Add logic to confirm the origin tag is visible again
                break;
            case MissionState::MISSION_COMPLETE:
                // TODO: Stop the robot
                break;
        }
    }

    // --- NEW: Publishing functions ---
    void publishState()
    {
        auto state_msg = std_msgs::msg::String();
        state_msg.data = missionStateToString(current_state_);
        state_pub_->publish(state_msg);
    }

    void publishDiagnostics()
    {
        auto diag_msg = std_msgs::msg::String();
        // TODO: Populate with real diagnostic data from the remote unit
        diag_msg.data = "Remote Unit Status: OK | Battery: 98% | Last Heartbeat: 12ms ago";
        diagnostics_pub_->publish(diag_msg);
    }

    // Member Variables
    MissionState current_state_ = MissionState::INITIALIZING;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tag_poses_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // --- NEW: Publisher member variables ---
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}