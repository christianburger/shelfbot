#include "shelfbot/behavior_tree_nodes/spin_action.hpp"

namespace BTNodes
{

SpinAction::SpinAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/four_wheel_drive_controller/cmd_vel", 10);
}

BT::PortsList SpinAction::providedPorts()
{
    return {
        BT::InputPort<double>("speed", 0.5, "Angular speed (rad/s)"),
        BT::InputPort<double>("duration", 10.0, "Spin duration (seconds)")
    };
}

BT::NodeStatus SpinAction::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "SpinAction: onStart");
    start_time_ = node_->get_clock()->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinAction::onRunning()
{
    double duration = getInput<double>("duration").value();
    if (node_->get_clock()->now() - start_time_ > rclcpp::Duration::from_seconds(duration))
    {
        RCLCPP_INFO(node_->get_logger(), "Spin duration elapsed. Failing.");
        // Stop the robot
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        return BT::NodeStatus::FAILURE;
    }

    // Publish spin command
    double speed = getInput<double>("speed").value();
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.angular.z = speed;
    cmd_vel_pub_->publish(twist_msg);

    return BT::NodeStatus::RUNNING;
}

void SpinAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "SpinAction: onHalted. Stopping robot.");
    // Ensure the robot stops if the action is halted
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);
}

} // namespace BTNodes
