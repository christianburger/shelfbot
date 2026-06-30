
// ─────────────────────────────────────────────────────────────────────────────
// bt_behaviors.cpp  —  Implementation of every custom BehaviorTree.CPP node.
//
// Threading contract (recap from header)
// ─────────────────────────────────────────────────────────────────────────────
// • BT tick loop  →  main thread
// • ROS2 executor →  background thread (spins the shared BtContext::node)
//
// Every service call follows the same pattern:
//   onStart()    — fire async_send_request, save SharedFuture, return RUNNING
//   onRunning()  — poll future::wait_for(0ms); return RUNNING until ready,
//                  then parse result and return SUCCESS/FAILURE
//   onHalted()   — reset future (we cannot cancel a service call in ROS2 but
//                  we discard the pending future so onRunning won't read it)
//
// Nav2 action calls follow the same pattern but using the action client's
// goal_response and result callbacks, which fire on the background thread.
// Access to goal_handle_ / nav_result_ is guarded by a mutex_.
// ─────────────────────────────────────────────────────────────────────────────

#include "shelfbot/bt_behaviors.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

namespace shelfbot::exploration {

// ─── blackboard key names ─────────────────────────────────────────────────────
static constexpr const char* BB_CTX          = "ctx";
static constexpr const char* BB_FRONTIER_ID  = "frontier_id";
static constexpr const char* BB_TARGET_X     = "target_x";
static constexpr const char* BB_TARGET_Y     = "target_y";
static constexpr const char* BB_NAV_SUCCEEDED = "nav_succeeded";

// ─────────────────────────────────────────────────────────────────────────────
// Internal helper: retrieve BtContext from the blackboard.
// ─────────────────────────────────────────────────────────────────────────────
static BtContextPtr ctx_from(const BT::NodeConfig& cfg) {
    return cfg.blackboard->get<BtContextPtr>(BB_CTX);
}

// ═════════════════════════════════════════════════════════════════════════════
// AllTargetTagsFound
// ═════════════════════════════════════════════════════════════════════════════
AllTargetTagsFound::AllTargetTagsFound(const std::string& name,
                                       const BT::NodeConfig& cfg)
    : BT::ConditionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::NodeStatus AllTargetTagsFound::tick() {
    using SrvT = shelfbot::srv::CheckTagsFound;
    auto& client = ctx_->check_tags;

    const auto now = std::chrono::steady_clock::now();

    // Re-use the cached result while it is still fresh
    if ((now - last_call_) < kCacheMs && !future_.has_value()) {
        return cached_status_;
    }

    // Fire a new request if none pending
    if (!future_.has_value()) {
        auto req       = std::make_shared<SrvT::Request>();
        req->target_ids = ctx_->target_tag_ids;
        future_    = client->async_send_request(req);
        last_call_ = now;
        // Return last known state while we wait (optimistic: assume not done yet)
        return cached_status_;
    }

    // Check if the response has arrived
    if (future_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return cached_status_;
    }

    auto res       = future_->get();
    future_.reset();
    cached_status_ = res->all_found ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

    if (res->all_found) {
        RCLCPP_INFO(ctx_->node->get_logger(),
            "[BT] AllTargetTagsFound: all %zu tags found — mission SUCCESS.",
            ctx_->target_tag_ids.size());
    }
    return cached_status_;
}

// ═════════════════════════════════════════════════════════════════════════════
// HasPendingFrontiers
// ═════════════════════════════════════════════════════════════════════════════
HasPendingFrontiers::HasPendingFrontiers(const std::string& name,
                                         const BT::NodeConfig& cfg)
    : BT::ConditionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::NodeStatus HasPendingFrontiers::tick() {
    using SrvT = shelfbot::srv::GetQueueSummary;
    auto& client = ctx_->get_summary;

    const auto now = std::chrono::steady_clock::now();
    if ((now - last_call_) < kCacheMs && !future_.has_value()) {
        return cached_status_;
    }

    if (!future_.has_value()) {
        future_    = client->async_send_request(std::make_shared<SrvT::Request>());
        last_call_ = now;
        return cached_status_;
    }

    if (future_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return cached_status_;
    }

    auto res       = future_->get();
    future_.reset();
    cached_status_ = res->has_pending ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

    if (!res->has_pending) {
        RCLCPP_INFO(ctx_->node->get_logger(),
            "[BT] HasPendingFrontiers: queue exhausted "
            "(pending=%d, succeeded=%d, blocked=%d).",
            res->pending, res->succeeded, res->blocked);
    }
    return cached_status_;
}

// ═════════════════════════════════════════════════════════════════════════════
// GetNextFrontierAction
// ═════════════════════════════════════════════════════════════════════════════
GetNextFrontierAction::GetNextFrontierAction(const std::string& name,
                                             const BT::NodeConfig& cfg)
    : BT::StatefulActionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::PortsList GetNextFrontierAction::providedPorts() {
    return {
        BT::OutputPort<int32_t>("frontier_id"),
        BT::OutputPort<double>("target_x"),
        BT::OutputPort<double>("target_y"),
    };
}

BT::NodeStatus GetNextFrontierAction::onStart() {
    using SrvT = shelfbot::srv::GetNextFrontier;
    future_ = ctx_->get_next->async_send_request(std::make_shared<SrvT::Request>());

    // Pre-set nav_succeeded = false on the blackboard.  If this action is
    // halted before NavigateToGoal has a chance to write the real result, the
    // downstream UpdateFrontierStatus will still find a valid value.
    config().blackboard->set<bool>(BB_NAV_SUCCEEDED, false);

    RCLCPP_DEBUG(ctx_->node->get_logger(), "[BT] GetNextFrontier: request sent.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextFrontierAction::onRunning() {
    if (!future_.has_value()) return BT::NodeStatus::FAILURE;
    if (future_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto res = future_->get();
    future_.reset();

    if (!res->success) {
        RCLCPP_WARN(ctx_->node->get_logger(),
            "[BT] GetNextFrontier: no frontier available (%s).", res->reason.c_str());
        return BT::NodeStatus::FAILURE;
    }

    setOutput("frontier_id", res->frontier_id);
    setOutput("target_x",    res->x);
    setOutput("target_y",    res->y);

    RCLCPP_INFO(ctx_->node->get_logger(),
        "[BT] GetNextFrontier: frontier %d at (%.2f, %.2f).",
        res->frontier_id, res->x, res->y);
    return BT::NodeStatus::SUCCESS;
}

void GetNextFrontierAction::onHalted() {
    future_.reset();
}

// ═════════════════════════════════════════════════════════════════════════════
// NavigateToGoalAction
// ═════════════════════════════════════════════════════════════════════════════
NavigateToGoalAction::NavigateToGoalAction(const std::string& name,
                                           const BT::NodeConfig& cfg)
    : BT::StatefulActionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::PortsList NavigateToGoalAction::providedPorts() {
    return {
        BT::InputPort<double>("target_x"),
        BT::InputPort<double>("target_y"),
        BT::OutputPort<bool>("nav_succeeded"),
    };
}

BT::NodeStatus NavigateToGoalAction::onStart() {
    using Nav2 = nav2_msgs::action::NavigateToPose;

    double tx{}, ty{};
    getInput("target_x", tx);
    getInput("target_y", ty);

    {
        std::lock_guard<std::mutex> lock(mtx_);
        goal_handle_.reset();
        nav_result_.reset();
        goal_accepted_.store(false);
        goal_rejected_.store(false);
    }

    Nav2::Goal goal;
    goal.pose.header.frame_id        = "map";
    goal.pose.header.stamp           = ctx_->node->now();
    goal.pose.pose.position.x        = tx;
    goal.pose.pose.position.y        = ty;
    goal.pose.pose.orientation.w     = 1.0;

    auto opts = rclcpp_action::Client<Nav2>::SendGoalOptions();

    opts.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<Nav2>::SharedPtr& gh) {
            std::lock_guard<std::mutex> lock(mtx_);
            if (gh) {
                goal_handle_ = gh;
                goal_accepted_.store(true);
                RCLCPP_DEBUG(ctx_->node->get_logger(),
                    "[BT] NavigateToGoal: goal accepted by Nav2.");
            } else {
                goal_rejected_.store(true);
                RCLCPP_WARN(ctx_->node->get_logger(),
                    "[BT] NavigateToGoal: goal REJECTED by Nav2.");
            }
        };

    opts.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<Nav2>::WrappedResult& result) {
            std::lock_guard<std::mutex> lock(mtx_);
            nav_result_ = result;
            RCLCPP_INFO(ctx_->node->get_logger(),
                "[BT] NavigateToGoal: result received (code=%d).",
                static_cast<int>(result.code));
        };

    ctx_->navigate->async_send_goal(goal, opts);

    RCLCPP_INFO(ctx_->node->get_logger(),
        "[BT] NavigateToGoal: sending goal to (%.2f, %.2f).", tx, ty);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToGoalAction::onRunning() {
    // Goal was rejected before being accepted
    if (goal_rejected_.load()) {
        setOutput("nav_succeeded", false);
        config().blackboard->set<bool>(BB_NAV_SUCCEEDED, false);
        return BT::NodeStatus::FAILURE;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (!nav_result_.has_value()) return BT::NodeStatus::RUNNING;

    const bool ok = (nav_result_->code == rclcpp_action::ResultCode::SUCCEEDED);
    setOutput("nav_succeeded", ok);
    config().blackboard->set<bool>(BB_NAV_SUCCEEDED, ok);

    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NavigateToGoalAction::onHalted() {
    RCLCPP_INFO(ctx_->node->get_logger(),
        "[BT] NavigateToGoal: halted — cancelling Nav2 goal.");
    std::lock_guard<std::mutex> lock(mtx_);
    if (goal_handle_) {
        ctx_->navigate->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// SpinAndScanAction
// ═════════════════════════════════════════════════════════════════════════════
SpinAndScanAction::SpinAndScanAction(const std::string& name,
                                     const BT::NodeConfig& cfg)
    : BT::StatefulActionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::NodeStatus SpinAndScanAction::onStart() {
    spin_start_ = ctx_->node->now();
    RCLCPP_INFO(ctx_->node->get_logger(),
        "[BT] SpinAndScan: starting %.1f s spin at %.2f rad/s.",
        ctx_->spin_duration_s, ctx_->spin_angular_vel);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinAndScanAction::onRunning() {
    const double elapsed = (ctx_->node->now() - spin_start_).seconds();

    if (elapsed >= ctx_->spin_duration_s) {
        // Spin complete — stop the robot
        ctx_->cmd_vel_pub->publish(geometry_msgs::msg::Twist{});
        RCLCPP_INFO(ctx_->node->get_logger(), "[BT] SpinAndScan: complete.");
        return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::Twist twist;
    twist.angular.z = ctx_->spin_angular_vel;
    ctx_->cmd_vel_pub->publish(twist);
    return BT::NodeStatus::RUNNING;
}

void SpinAndScanAction::onHalted() {
    // Ensure the robot stops even if we are halted mid-spin
    ctx_->cmd_vel_pub->publish(geometry_msgs::msg::Twist{});
    RCLCPP_INFO(ctx_->node->get_logger(), "[BT] SpinAndScan: halted — robot stopped.");
}

// ═════════════════════════════════════════════════════════════════════════════
// MarkFrontierBlockedAction
// ═════════════════════════════════════════════════════════════════════════════
MarkFrontierBlockedAction::MarkFrontierBlockedAction(const std::string& name,
                                                     const BT::NodeConfig& cfg)
    : BT::SyncActionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::PortsList MarkFrontierBlockedAction::providedPorts() {
    return {
        BT::InputPort<int32_t>("frontier_id"),
        BT::OutputPort<bool>("nav_succeeded"),
    };
}

BT::NodeStatus MarkFrontierBlockedAction::tick() {
    // Mark the blackboard so UpdateFrontierStatus sends BLOCKED to the queue
    setOutput("nav_succeeded", false);
    config().blackboard->set<bool>(BB_NAV_SUCCEEDED, false);

    int32_t fid{-1};
    getInput("frontier_id", fid);
    RCLCPP_INFO(ctx_->node->get_logger(),
        "[BT] MarkFrontierBlocked: nav failed for frontier %d.", fid);

    // Always return SUCCESS so the outer Selector moves on to UpdateFrontierStatus
    return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════════════════
// UpdateFrontierStatusAction
// ═════════════════════════════════════════════════════════════════════════════
UpdateFrontierStatusAction::UpdateFrontierStatusAction(const std::string& name,
                                                       const BT::NodeConfig& cfg)
    : BT::StatefulActionNode(name, cfg)
    , ctx_(ctx_from(cfg))
{}

BT::PortsList UpdateFrontierStatusAction::providedPorts() {
    return {
        BT::InputPort<int32_t>("frontier_id"),
        BT::InputPort<bool>("nav_succeeded"),
    };
}

BT::NodeStatus UpdateFrontierStatusAction::onStart() {
    using SrvT = shelfbot::srv::UpdateFrontierStatus;

    int32_t fid{-1};
    bool    nav_ok{false};
    getInput("frontier_id",  fid);
    getInput("nav_succeeded", nav_ok);

    // Also read directly from the blackboard as a fallback for the case where
    // the port value was never explicitly set (e.g. when halted mid-nav)
    if (!nav_ok) {
        config().blackboard->get<bool>(BB_NAV_SUCCEEDED, nav_ok);
    }

    const std::string new_status = nav_ok ? "SUCCEEDED" : "BLOCKED";

    auto req          = std::make_shared<SrvT::Request>();
    req->frontier_id  = fid;
    req->status       = new_status;
    future_           = ctx_->update_status->async_send_request(req);

    RCLCPP_INFO(ctx_->node->get_logger(),
        "[BT] UpdateFrontierStatus: frontier %d → %s.",
        fid, new_status.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UpdateFrontierStatusAction::onRunning() {
    if (!future_.has_value()) return BT::NodeStatus::SUCCESS;  // halted path
    if (future_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }
    // We always succeed here — a missed update is handled by the queue's
    // stale-ACTIVE requeue timer rather than failing the whole mission cycle.
    future_.reset();
    return BT::NodeStatus::SUCCESS;
}

void UpdateFrontierStatusAction::onHalted() {
    future_.reset();
}

// ═════════════════════════════════════════════════════════════════════════════
// Factory registration
// ═════════════════════════════════════════════════════════════════════════════
void register_exploration_nodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<AllTargetTagsFound>    ("AllTargetTagsFound");
    factory.registerNodeType<HasPendingFrontiers>   ("HasPendingFrontiers");
    factory.registerNodeType<GetNextFrontierAction> ("GetNextFrontier");
    factory.registerNodeType<NavigateToGoalAction>  ("NavigateToGoal");
    factory.registerNodeType<SpinAndScanAction>     ("SpinAndScan");
    factory.registerNodeType<MarkFrontierBlockedAction>("MarkFrontierBlocked");
    factory.registerNodeType<UpdateFrontierStatusAction>("UpdateFrontierStatus");
}

}  // namespace shelfbot::exploration
