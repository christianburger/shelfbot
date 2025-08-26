#ifndef SHELF_BT_SPIN_ACTION_HPP
#define SHELF_BT_SPIN_ACTION_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace BTNodes
{

class SpinAction : public BT::StatefulActionNode
{
public:
    SpinAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    // Method overrides
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Define the ports for this node
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Time start_time_;
};

} // namespace BTNodes

#endif // SHELF_BT_SPIN_ACTION_HPP
