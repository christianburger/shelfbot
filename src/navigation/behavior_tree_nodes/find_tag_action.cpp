#include "shelfbot/behavior_tree_nodes/find_tag_action.hpp"

namespace BTNodes
{

FindTagAction::FindTagAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr tf_buffer)
    : BT::StatefulActionNode(name, config), node_(node), tf_buffer_(tf_buffer)
{
}

BT::PortsList FindTagAction::providedPorts()
{
    return {
        BT::InputPort<int>("tag_id", "The integer ID of the tag to find"),
        BT::InputPort<std::string>("tag_frame_name", "The name of the TF frame for the tag to find"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "The found pose of the tag")
    };
}

BT::NodeStatus FindTagAction::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "FindTagAction: onStart");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FindTagAction::onRunning()
{
    std::string tag_frame = getInput<std::string>("tag_frame_name").value();
    geometry_msgs::msg::TransformStamped transform;

    try
    {
        // Check if the transform from the robot's base to the tag is available
        transform = tf_buffer_->lookupTransform("base_link", tag_frame, tf2::TimePointZero);
        
        RCLCPP_INFO(node_->get_logger(), "Found tag '%s' relative to base_link!", tag_frame.c_str());

        // If found, write the pose to the blackboard
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->get_clock()->now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        
        setOutput("output_pose", pose);
        return BT::NodeStatus::SUCCESS;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_INFO(node_->get_logger(), "Could not find transform to '%s': %s", tag_frame.c_str(), ex.what());
        // Return RUNNING to keep trying on the next tick
        return BT::NodeStatus::RUNNING;
    }
}

void FindTagAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "FindTagAction: onHalted");
}

} // namespace BTNodes
