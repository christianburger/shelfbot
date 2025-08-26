#ifndef SHELF_BT_MOVE_ACTION_HPP
#define SHELF_BT_MOVE_ACTION_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// --- FIX: The specialization must be in the BT namespace ---
namespace BT
{
// Custom type converter for geometry_msgs::msg::Twist
// It can be used to parse a string like "lx;ly;lz:ax;ay;az"
template <> inline geometry_msgs::msg::Twist convertFromString(StringView str)
{
    auto parts = splitString(str, ':');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }

    auto linear_parts = splitString(parts[0], ';');
    auto angular_parts = splitString(parts[1], ';');

    if (linear_parts.size() != 3 || angular_parts.size() != 3)
    {
        throw RuntimeError("invalid input)");
    }

    geometry_msgs::msg::Twist output;
    output.linear.x = convertFromString<double>(linear_parts[0]);
    output.linear.y = convertFromString<double>(linear_parts[1]);
    output.linear.z = convertFromString<double>(linear_parts[2]);
    output.angular.x = convertFromString<double>(angular_parts[0]);
    output.angular.y = convertFromString<double>(angular_parts[1]);
    output.angular.z = convertFromString<double>(angular_parts[2]);

    return output;
}
} // namespace BT


namespace BTNodes
{

class MoveAction : public BT::StatefulActionNode
{
public:
    MoveAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

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

#endif // SHELF_BT_MOVE_ACTION_HPP