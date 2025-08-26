#ifndef SHELF_BT_FIND_TAG_ACTION_HPP
#define SHELF_BT_FIND_TAG_ACTION_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BTNodes
{

class FindTagAction : public BT::StatefulActionNode
{
public:
    FindTagAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr tf_buffer);

    // Method overrides
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Define the ports for this node
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer::SharedPtr tf_buffer_;
};

} // namespace BTNodes

#endif // SHELF_BT_FIND_TAG_ACTION_HPP
