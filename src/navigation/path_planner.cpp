#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Include our custom BT node headers
#include "shelfbot/behavior_tree_nodes/find_tag_action.hpp"
#include "shelfbot/behavior_tree_nodes/spin_action.hpp"
#include "shelfbot/behavior_tree_nodes/move_action.hpp"


class MissionControlNode : public rclcpp::Node
{
public:
    MissionControlNode() : Node("mission_control_node")
    {
    }

    void setup()
    {
        RCLCPP_INFO(this->get_logger(), "--- Mission Control Node Initialization ---");

        RCLCPP_INFO(this->get_logger(), "[1/7] Initializing TF buffer and listener...");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "      ...TF initialized successfully.");

        RCLCPP_INFO(this->get_logger(), "[2/7] Creating Behavior Tree factory...");
        BT::BehaviorTreeFactory factory;
        RCLCPP_INFO(this->get_logger(), "      ...Factory created successfully.");

        RCLCPP_INFO(this->get_logger(), "[3/7] Registering custom Behavior Tree nodes...");
        factory.registerNodeType<BTNodes::FindTagAction>("FindTag", shared_from_this(), tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "      ...Registered 'FindTagAction'.");
        factory.registerNodeType<BTNodes::SpinAction>("Spin", shared_from_this());
        RCLCPP_INFO(this->get_logger(), "      ...Registered 'SpinAction'.");
        factory.registerNodeType<BTNodes::MoveAction>("MoveAction", shared_from_this());
        RCLCPP_INFO(this->get_logger(), "      ...Registered 'MoveAction'.");

        RCLCPP_INFO(this->get_logger(), "[4/7] Locating and loading Behavior Tree XML file...");
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("shelfbot");
        std::string xml_file = package_share_directory + "/config/mission.xml";
        RCLCPP_INFO(this->get_logger(), "      ...Loading from: %s", xml_file.c_str());
        try
        {
            tree_ = factory.createTreeFromFile(xml_file);
            RCLCPP_INFO(this->get_logger(), "      ...Tree created successfully from file.");
        }
        catch (const BT::RuntimeError& e)
        {
            RCLCPP_ERROR(this->get_logger(), "FATAL: Failed to create Behavior Tree: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "[5/7] Creating BT console logger...");
        logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
        RCLCPP_INFO(this->get_logger(), "      ...Logger created successfully.");

        RCLCPP_INFO(this->get_logger(), "[6/7] Creating BT ticker timer (10 Hz)...");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionControlNode::tickTree, this));
        RCLCPP_INFO(this->get_logger(), "      ...Timer created successfully.");

        RCLCPP_INFO(this->get_logger(), "[7/7] --- Initialization Complete ---");
    }

private:
    void tickTree()
    {
        BT::NodeStatus status = tree_.tickOnce();
        if (status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Mission SUCCESS");
            timer_->cancel();
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(this->get_logger(), "Mission FAILURE");
            timer_->cancel();
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<BT::StdCoutLogger> logger_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionControlNode>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}