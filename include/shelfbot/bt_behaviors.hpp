#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// bt_behaviors.hpp  —  Declaration of every custom BehaviorTree.CPP node used
// by the exploration mission.
//
// Tree structure reminder
// ─────────────────────────────────────────────────────────────────────────────
//
//  Selector("MissionRoot")                     ← no-memory: re-checks gates every tick
//  ├── AllTargetTagsFound                       CONDITION → SUCCESS when done
//  ├── Inverter → HasPendingFrontiers           CONDITION → SUCCESS when queue empty
//  └── Sequence("FrontierCycle")               ← memory: resumes across ticks
//      ├── GetNextFrontier                      ACTION    pops queue → bb
//      ├── Selector("NavWithRecovery")
//      │   ├── Sequence("NavigateAndScan")
//      │   │   ├── NavigateToGoal              ACTION    Nav2 action, RUNNING
//      │   │   └── SpinAndScan                 ACTION    360° /cmd_vel spin
//      │   └── MarkFrontierBlocked             ACTION    nav failed → mark BLOCKED
//      └── UpdateFrontierStatus                ACTION    writes queue entry
//
// Threading model
// ─────────────────────────────────────────────────────────────────────────────
// The BT tick loop runs on the MAIN thread.
// A SingleThreadedExecutor spins the shared rclcpp::Node on a BACKGROUND thread.
// Service futures and Nav2 action callbacks are resolved on the background thread.
// BT behaviors poll future::wait_for(0ms) to avoid blocking the tick thread.
// Shared mutable state (goal_handle_, nav_result_) is guarded by std::mutex.
// ─────────────────────────────────────────────────────────────────────────────

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "shelfbot/bt_context.hpp"

namespace shelfbot::exploration {

// ─── convenience aliases ──────────────────────────────────────────────────────
using Nav2Action    = nav2_msgs::action::NavigateToPose;
using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Action>;

// ─────────────────────────────────────────────────────────────────────────────
// CONDITION: returns SUCCESS when all target_tag_ids are in the registry.
// Caches the last service response for cache_ms to avoid a service call on
// every single tick.
// ─────────────────────────────────────────────────────────────────────────────
class AllTargetTagsFound : public BT::ConditionNode {
public:
    AllTargetTagsFound(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;

private:
    BtContextPtr ctx_;
    using SrvT     = shelfbot::srv::CheckTagsFound;
    using FutureT  = rclcpp::Client<SrvT>::SharedFuture;

    std::optional<FutureT>  future_;
    BT::NodeStatus           cached_status_{BT::NodeStatus::FAILURE};
    std::chrono::steady_clock::time_point last_call_{};
    static constexpr std::chrono::milliseconds kCacheMs{500};
};

// ─────────────────────────────────────────────────────────────────────────────
// CONDITION: returns SUCCESS when the frontier queue has ≥ 1 PENDING frontier.
// Cached similarly to AllTargetTagsFound.
// ─────────────────────────────────────────────────────────────────────────────
class HasPendingFrontiers : public BT::ConditionNode {
public:
    HasPendingFrontiers(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;

private:
    BtContextPtr ctx_;
    using SrvT    = shelfbot::srv::GetQueueSummary;
    using FutureT = rclcpp::Client<SrvT>::SharedFuture;

    std::optional<FutureT>  future_;
    // Defaults to SUCCESS ("assume pending work exists") rather than
    // FAILURE, unlike AllTargetTagsFound above. Until the first real
    // /frontier_queue/get_summary response arrives, this placeholder value
    // is what tick() returns. If it defaulted to FAILURE (as it did
    // originally), Inverter(HasPendingFrontiers) would evaluate to SUCCESS
    // on the very first tick — before any real queue data is known — and
    // the root Fallback would conclude "mission complete, nothing to
    // explore" immediately, even with a full queue of pending frontiers.
    BT::NodeStatus           cached_status_{BT::NodeStatus::SUCCESS};
    std::chrono::steady_clock::time_point last_call_{};
    static constexpr std::chrono::milliseconds kCacheMs{500};
};

// ─────────────────────────────────────────────────────────────────────────────
// ACTION: pops the next PENDING frontier from the queue.
// On SUCCESS writes frontier_id, target_x, target_y to the blackboard and
// pre-sets nav_succeeded=false so the downstream UpdateFrontierStatus always
// has a valid value to read even if navigation is halted mid-flight.
// ─────────────────────────────────────────────────────────────────────────────
class GetNextFrontierAction : public BT::StatefulActionNode {
public:
    GetNextFrontierAction(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    BtContextPtr ctx_;
    using SrvT    = shelfbot::srv::GetNextFrontier;
    using FutureT = rclcpp::Client<SrvT>::SharedFuture;
    std::optional<FutureT> future_;
};

// ─────────────────────────────────────────────────────────────────────────────
// ACTION: sends a NavigateToPose goal to Nav2 and waits for the result.
// Returns SUCCESS on SUCCEEDED, FAILURE on ABORTED/CANCELED.
// On SUCCESS:  sets nav_succeeded=true  on the blackboard.
// On FAILURE:  sets nav_succeeded=false on the blackboard.
// onHalted cancels the in-flight Nav2 goal so the robot stops cleanly.
// ─────────────────────────────────────────────────────────────────────────────
class NavigateToGoalAction : public BT::StatefulActionNode {
public:
    NavigateToGoalAction(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    BtContextPtr ctx_;

    std::mutex                              mtx_;
    std::shared_ptr<Nav2GoalHandle>         goal_handle_;
    std::optional<Nav2GoalHandle::WrappedResult> nav_result_;
    std::atomic<bool>                       goal_accepted_{false};
    std::atomic<bool>                       goal_rejected_{false};
};

// ─────────────────────────────────────────────────────────────────────────────
// ACTION: rotates the robot in place by publishing to /cmd_vel.
// Duration and angular velocity are read from BtContext (set from parameters).
// Always returns SUCCESS after the spin completes.
// Publishes a zero-velocity Twist on halt to ensure the robot stops.
// ─────────────────────────────────────────────────────────────────────────────
class SpinAndScanAction : public BT::StatefulActionNode {
public:
    SpinAndScanAction(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    BtContextPtr ctx_;
    rclcpp::Time spin_start_;
};

// ─────────────────────────────────────────────────────────────────────────────
// ACTION (failure branch): called when NavigateToGoalAction fails.
// Sets nav_succeeded=false on the blackboard and always returns SUCCESS so the
// outer Selector can continue to UpdateFrontierStatus.
// ─────────────────────────────────────────────────────────────────────────────
class MarkFrontierBlockedAction : public BT::SyncActionNode {
public:
    MarkFrontierBlockedAction(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    BtContextPtr ctx_;
};

// ─────────────────────────────────────────────────────────────────────────────
// ACTION: reads frontier_id and nav_succeeded from the blackboard and calls
// frontier_queue/update_status with SUCCEEDED or BLOCKED accordingly.
// Always returns SUCCESS (status update is best-effort; a dropped update is
// not a mission failure — the queue's requeue timer will handle stale ACTIVE
// entries on the next cycle).
// ─────────────────────────────────────────────────────────────────────────────
class UpdateFrontierStatusAction : public BT::StatefulActionNode {
public:
    UpdateFrontierStatusAction(const std::string& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    BtContextPtr ctx_;
    using SrvT    = shelfbot::srv::UpdateFrontierStatus;
    using FutureT = rclcpp::Client<SrvT>::SharedFuture;
    std::optional<FutureT> future_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Factory helper: registers all six custom nodes with a BT::BehaviorTreeFactory.
// Call this once before creating the tree.
// ─────────────────────────────────────────────────────────────────────────────
void register_exploration_nodes(BT::BehaviorTreeFactory& factory);

}  // namespace shelfbot::exploration
