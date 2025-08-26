#include "shelfbot/behavior_tree_nodes/move_action.hpp"

namespace BTNodes
{

MoveAction::MoveAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/four_wheel_drive_controller/cmd_vel", 10);
}

BT::PortsList MoveAction::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::Twist>("twist", "The Twist command to publish"),
        BT::InputPort<double>("duration", 1.0, "Publish duration (seconds)")
    };
}

BT::NodeStatus MoveAction::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "MoveAction: onStart");
    start_time_ = node_->get_clock()->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveAction::onRunning()
{
    double duration = getInput<double>("duration").value();
    RCLCPP_INFO(node_->get_logger(), "MoveAction: onRunning (Duration: %.1f)", duration);

    if (node_->get_clock()->now() - start_time_ > rclcpp::Duration::from_seconds(duration))
    {
        RCLCPP_INFO(node_->get_logger(), "MoveAction: duration elapsed. Stopping robot and returning SUCCESS.");
        // Stop the robot
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        cmd_vel_pub_->publish(stop_msg);
        return BT::NodeStatus::SUCCESS;
    }

    // Get the Twist message from the input port and publish it
    auto twist = getInput<geometry_msgs::msg::Twist>("twist");
    if (!twist)
    {
        RCLCPP_ERROR(node_->get_logger(), "MoveAction: 'twist' port is required but not set. Returning FAILURE.");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "MoveAction: Publishing Twist (Lin: %.1f, Ang: %.1f)", twist.value().linear.x, twist.value().angular.z);
    cmd_vel_pub_->publish(twist.value());

    return BT::NodeStatus::RUNNING;
}

void MoveAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "MoveAction: onHalted. Stopping robot.");
    // Ensure the robot stops if the action is halted
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);
}

} // namespace BTNodes
