#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// bt_context.hpp  —  Single aggregate of every ROS2 resource the BT behaviors
// need.  Created once by ExplorationBTNode and stored on the BT blackboard
// under the key "ctx".
//
// Design rationale
// ─────────────────────────────────────────────────────────────────────────────
// Passing one shared_ptr<BtContext> via the blackboard avoids the alternative
// of each BT node needing its own rclcpp::Node, creating duplicate service
// clients, and managing separate lifetimes.  All clients are created once on
// a single rclcpp::Node; the BT behaviors are just thin wrappers that call
// those clients.
// ─────────────────────────────────────────────────────────────────────────────

#include <memory>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <shelfbot/srv/get_next_frontier.hpp>
#include <shelfbot/srv/update_frontier_status.hpp>
#include <shelfbot/srv/get_queue_summary.hpp>
#include <shelfbot/srv/check_tags_found.hpp>

namespace shelfbot::exploration {

struct BtContext {
    // ── shared node ───────────────────────────────────────────────────────────
    // All clients are created on this node.  The node is spun by a
    // SingleThreadedExecutor in a background thread so that service futures
    // and action callbacks resolve independently of the BT tick loop.
    rclcpp::Node::SharedPtr node;

    // ── frontier queue clients ────────────────────────────────────────────────
    rclcpp::Client<shelfbot::srv::GetNextFrontier>::SharedPtr      get_next;
    rclcpp::Client<shelfbot::srv::UpdateFrontierStatus>::SharedPtr update_status;
    rclcpp::Client<shelfbot::srv::GetQueueSummary>::SharedPtr      get_summary;

    // ── tag registry client ───────────────────────────────────────────────────
    rclcpp::Client<shelfbot::srv::CheckTagsFound>::SharedPtr       check_tags;

    // ── nav2 action client ────────────────────────────────────────────────────
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate;

    // ── cmd_vel publisher (used by SpinAndScan) ───────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    // ── mission parameters ────────────────────────────────────────────────────
    std::vector<int32_t> target_tag_ids;     // IDs we must find before stopping
    double               spin_angular_vel;   // [rad/s] rotation speed for SpinAndScan
    double               spin_duration_s;    // [s]     total spin time ≈ 2π/angular_vel

    // ── service availability helpers ──────────────────────────────────────────
    bool wait_for_services(std::chrono::seconds timeout = std::chrono::seconds(10)) const {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        auto wait_one = [&](auto& client, const char* name) -> bool {
            while (!client->wait_for_service(std::chrono::milliseconds(200))) {
                if (std::chrono::steady_clock::now() > deadline) {
                    RCLCPP_ERROR(node->get_logger(),
                        "Timed out waiting for service: %s", name);
                    return false;
                }
            }
            return true;
        };
        return wait_one(get_next,      "frontier_queue/get_next")
            && wait_one(update_status, "frontier_queue/update_status")
            && wait_one(get_summary,   "frontier_queue/get_summary")
            && wait_one(check_tags,    "tag_registry/check_found");
    }
};

using BtContextPtr = std::shared_ptr<BtContext>;

}  // namespace shelfbot::exploration
