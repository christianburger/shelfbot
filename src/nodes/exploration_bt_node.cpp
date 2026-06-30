// ─────────────────────────────────────────────────────────────────────────────
// exploration_bt_node.cpp
//
// Owns the BtContext, builds the BT tree from config/exploration_tree.xml,
// and ticks it at a fixed rate on the main thread while the shared
// rclcpp::Node spins on a background thread.
//
// This node is launched as part of the mission tier of real_robot_nav2.launch.py
// (see launch/real_robot_nav2.launch.py). It assumes frontier_discovery_node,
// frontier_queue_node, tag_registry_node, and Nav2's bt_navigator are already
// up and serving /frontier_queue/*, /tag_registry/check_found, and
// /navigate_to_pose respectively.
// ─────────────────────────────────────────────────────────────────────────────
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>   // optional Groot2 live view

#include "shelfbot/bt_context.hpp"
#include "shelfbot/bt_behaviors.hpp"

using namespace shelfbot::exploration;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("exploration_bt_node");

    // ── declare mission parameters ──────────────────────────────────────────
    node->declare_parameter("target_tag_ids", std::vector<int64_t>{});
    node->declare_parameter("spin_angular_vel", 0.6);     // rad/s
    node->declare_parameter("spin_duration_s", 10.5);     // ≈ 2π / 0.6
    node->declare_parameter("tick_period_ms", 200);
    node->declare_parameter("bt_xml_path", std::string(""));  // empty = use default
    node->declare_parameter("enable_groot_monitoring", false);

    // ── build BtContext ──────────────────────────────────────────────────────
    auto ctx = std::make_shared<BtContext>();
    ctx->node = node;

    auto tag_ids_param = node->get_parameter("target_tag_ids").as_integer_array();
    ctx->target_tag_ids.assign(tag_ids_param.begin(), tag_ids_param.end());
    ctx->spin_angular_vel = node->get_parameter("spin_angular_vel").as_double();
    ctx->spin_duration_s  = node->get_parameter("spin_duration_s").as_double();

    ctx->get_next      = node->create_client<shelfbot::srv::GetNextFrontier>(
        "/frontier_queue/get_next");
    ctx->update_status = node->create_client<shelfbot::srv::UpdateFrontierStatus>(
        "/frontier_queue/update_status");
    ctx->get_summary   = node->create_client<shelfbot::srv::GetQueueSummary>(
        "/frontier_queue/get_summary");
    ctx->check_tags    = node->create_client<shelfbot::srv::CheckTagsFound>(
        "/tag_registry/check_found");

    ctx->navigate = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node, "navigate_to_pose");

    ctx->cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    RCLCPP_INFO(node->get_logger(), "Waiting for frontier_queue / tag_registry services...");
    if (!ctx->wait_for_services(std::chrono::seconds(30))) {
        RCLCPP_FATAL(node->get_logger(), "Required services never appeared. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "All services available.");

    if (!ctx->navigate->wait_for_action_server(std::chrono::seconds(30))) {
        RCLCPP_FATAL(node->get_logger(), "navigate_to_pose action server never appeared.");
        rclcpp::shutdown();
        return 1;
    }

    // ── spin the shared node on a background thread ────────────────────────
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // ── build the tree ───────────────────────────────────────────────────────
    BT::BehaviorTreeFactory factory;
    register_exploration_nodes(factory);

    auto blackboard = BT::Blackboard::create();
    blackboard->set<BtContextPtr>("ctx", ctx);

    std::string xml_path = node->get_parameter("bt_xml_path").as_string();
    if (xml_path.empty()) {
        xml_path = ament_index_cpp::get_package_share_directory("shelfbot")
                 + "/config/exploration_tree.xml";
    }
    RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());

    BT::Tree tree = factory.createTreeFromFile(xml_path, blackboard);

    // Optional: live Groot2 monitoring (publishes on default ZMQ ports)
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher;
    if (node->get_parameter("enable_groot_monitoring").as_bool()) {
        zmq_publisher = std::make_unique<BT::PublisherZMQ>(tree);
        RCLCPP_INFO(node->get_logger(), "Groot2 monitoring enabled on default ports.");
    }

    // ── tick loop on the MAIN thread ────────────────────────────────────────
    const int period_ms = node->get_parameter("tick_period_ms").as_int();
    rclcpp::WallRate rate(std::chrono::milliseconds(period_ms));

    RCLCPP_INFO(node->get_logger(), "Exploration BT started. Ticking every %d ms.", period_ms);

    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(),
                "Exploration mission complete (SUCCESS). Halting tree.");
            tree.haltTree();
            break;
        }
        if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_WARN(node->get_logger(),
                "Exploration tree returned FAILURE at root — this should not "
                "happen with the documented tree structure (root Selector "
                "always has a fallback). Check XML wiring.");
            // Keep ticking rather than exiting — a transient service hiccup
            // should not kill the mission node.
        }

        rate.sleep();
    }

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
