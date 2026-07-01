
// ─────────────────────────────────────────────────────────────────────────────
// frontier_queue_node.cpp
//
// INDEPENDENT node.  No navigation or tag knowledge.
// Maintains a live, stateful registry of frontiers discovered by
// frontier_discovery_node, and exposes three services consumed by the BT.
//
// Merge strategy
//   New poses from /frontiers are added only when they are further than
//   `merge_radius` from every existing entry — including BLOCKED ones.
//   This prevents the queue from growing unboundedly with near-duplicate spots.
//
// Requeue strategy
//   BLOCKED entries whose attempt count is below `max_attempts` are
//   automatically reset to PENDING after `requeue_delay_s` seconds.
//
// ─── Interfaces ───────────────────────────────────────────────────────────────
// Subscribe  /frontiers                 geometry_msgs/PoseArray
// Publish    /frontier_queue/markers    visualization_msgs/MarkerArray
// Service    ~/get_next                 shelfbot/srv/GetNextFrontier
// Service    ~/update_status            shelfbot/srv/UpdateFrontierStatus
// Service    ~/get_summary              shelfbot/srv/GetQueueSummary
// ─────────────────────────────────────────────────────────────────────────────
#include <map>
#include <mutex>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>

#include <shelfbot/srv/get_next_frontier.hpp>
#include <shelfbot/srv/update_frontier_status.hpp>
#include <shelfbot/srv/get_queue_summary.hpp>

#include "shelfbot/types.hpp"

using namespace shelfbot::exploration;

// ─────────────────────────────────────────────────────────────────────────────
class FrontierQueueNode : public rclcpp::Node {
public:
    explicit FrontierQueueNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("frontier_queue", opts)
    {
        declare_parameter("merge_radius",    0.40);  // [m]
        declare_parameter("max_attempts",    3);
        declare_parameter("requeue_delay_s", 30.0);
        declare_parameter("publish_hz",      2.0);

        frontiers_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/frontiers", 10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                on_frontiers(msg);
            });

        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontier_queue/markers", 10);

        svc_next_ = create_service<shelfbot::srv::GetNextFrontier>(
            "~/get_next",
            [this](const shelfbot::srv::GetNextFrontier::Request::SharedPtr  req,
                         shelfbot::srv::GetNextFrontier::Response::SharedPtr res) {
                (void)req;
                handle_get_next(res);
            });

        svc_update_ = create_service<shelfbot::srv::UpdateFrontierStatus>(
            "~/update_status",
            [this](const shelfbot::srv::UpdateFrontierStatus::Request::SharedPtr  req,
                         shelfbot::srv::UpdateFrontierStatus::Response::SharedPtr res) {
                handle_update(req, res);
            });

        svc_summary_ = create_service<shelfbot::srv::GetQueueSummary>(
            "~/get_summary",
            [this](const shelfbot::srv::GetQueueSummary::Request::SharedPtr  req,
                         shelfbot::srv::GetQueueSummary::Response::SharedPtr res) {
                (void)req;
                handle_summary(res);
            });

        const double period = 1.0 / get_parameter("publish_hz").as_double();
        marker_timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            [this] { publish_markers(); });

        // Requeue check runs on a 5 s interval — coarser than the visualisation.
        requeue_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this] { requeue_stale(); });

        RCLCPP_INFO(get_logger(), "FrontierQueue ready.");
    }

private:
    // ── frontier ingestion ────────────────────────────────────────────────────
    void on_frontiers(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        const double merge_r = get_parameter("merge_radius").as_double();
        int added = 0;

        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& pose : msg->poses) {
            const double nx = pose.position.x;
            const double ny = pose.position.y;

            // Skip if within merge_radius of any existing entry
            bool duplicate = false;
            for (const auto& [id, e] : entries_) {
                if (e.distance_to(nx, ny) < merge_r) { duplicate = true; break; }
            }
            if (duplicate) continue;

            FrontierEntry entry;
            entry.id = next_id_++;
            entry.x  = nx;
            entry.y  = ny;
            entries_.emplace(entry.id, entry);
            ++added;
        }

        if (added > 0) {
            RCLCPP_INFO(get_logger(),
                "Added %d new frontiers (total: %zu).", added, entries_.size());
        }
    }

    // ── service: get_next ─────────────────────────────────────────────────────
    void handle_get_next(shelfbot::srv::GetNextFrontier::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Find oldest PENDING entry (lowest id = earliest inserted)
        FrontierEntry* selected = nullptr;
        for (auto& [id, e] : entries_) {
            if (e.status == FrontierStatus::PENDING) {
                selected = &e;
                break;  // map is ordered by id, so first hit is oldest
            }
        }

        if (!selected) {
            res->frontier_id = -1;
            res->success     = false;
            res->reason      = "No PENDING frontiers in queue.";
            return;
        }

        selected->status     = FrontierStatus::ACTIVE;
        selected->attempts  += 1;
        selected->updated_at = FrontierEntry::Clock::now();

        res->frontier_id = selected->id;
        res->x           = selected->x;
        res->y           = selected->y;
        res->success     = true;
        res->reason      = "Frontier " + std::to_string(selected->id)
                         + " marked ACTIVE (attempt "
                         + std::to_string(selected->attempts) + ").";

        RCLCPP_INFO(get_logger(), "%s", res->reason.c_str());
    }

    // ── service: update_status ────────────────────────────────────────────────
    void handle_update(
            const shelfbot::srv::UpdateFrontierStatus::Request::SharedPtr  req,
                  shelfbot::srv::UpdateFrontierStatus::Response::SharedPtr res) {
        FrontierStatus new_status{};
        if (!from_string(req->status, new_status)) {
            res->success = false;
            res->reason  = "Unknown status string: " + req->status;
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = entries_.find(req->frontier_id);
        if (it == entries_.end()) {
            res->success = false;
            res->reason  = "Frontier " + std::to_string(req->frontier_id)
                         + " not found.";
            return;
        }

        it->second.status     = new_status;
        it->second.updated_at = FrontierEntry::Clock::now();

        res->success = true;
        res->reason  = "Frontier " + std::to_string(req->frontier_id)
                     + " → " + req->status + ".";

        RCLCPP_INFO(get_logger(), "%s", res->reason.c_str());
    }

    // ── service: get_summary ──────────────────────────────────────────────────
    void handle_summary(shelfbot::srv::GetQueueSummary::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [id, e] : entries_) {
            switch (e.status) {
                case FrontierStatus::PENDING:   ++res->pending;   break;
                case FrontierStatus::ACTIVE:    ++res->active;    break;
                case FrontierStatus::SUCCEEDED: ++res->succeeded; break;
                case FrontierStatus::BLOCKED:   ++res->blocked;   break;
            }
        }
        res->total       = static_cast<int32_t>(entries_.size());
        res->has_pending = res->pending > 0;
    }

    // ── requeue ───────────────────────────────────────────────────────────────
    void requeue_stale() {
        const int    max_att  = get_parameter("max_attempts").as_int();
        const double delay_s  = get_parameter("requeue_delay_s").as_double();
        const auto   delay    = std::chrono::duration<double>(delay_s);
        const auto   now      = FrontierEntry::Clock::now();
        int requeued = 0;

        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [id, e] : entries_) {
            if (e.status != FrontierStatus::BLOCKED)    continue;
            if (e.attempts >= max_att)                  continue;
            if ((now - e.updated_at) < delay)           continue;

            e.status     = FrontierStatus::PENDING;
            e.updated_at = now;
            ++requeued;
        }
        if (requeued > 0)
            RCLCPP_INFO(get_logger(), "Requeued %d BLOCKED frontiers.", requeued);
    }

    // ── visualisation ─────────────────────────────────────────────────────────
    void publish_markers() {
        visualization_msgs::msg::MarkerArray array;
        std_msgs::msg::Header hdr;
        hdr.frame_id = "map";
        hdr.stamp    = now();

        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        clear.header = hdr;
        array.markers.push_back(clear);

        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [id, e] : entries_) {
            auto [r, g, b] = e.status_color();

            // Sphere
            visualization_msgs::msg::Marker sphere;
            sphere.header          = hdr;
            sphere.ns              = "frontier_queue";
            sphere.id              = e.id;
            sphere.type            = visualization_msgs::msg::Marker::SPHERE;
            sphere.action          = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = e.x;
            sphere.pose.position.y = e.y;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20;
            sphere.color.r = r; sphere.color.g = g;
            sphere.color.b = b; sphere.color.a = 0.85f;
            array.markers.push_back(sphere);

            // Text label
            visualization_msgs::msg::Marker label;
            label.header          = hdr;
            label.ns              = "frontier_queue_labels";
            label.id              = e.id;
            label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action          = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x = e.x;
            label.pose.position.y = e.y;
            label.pose.position.z = 0.28;
            label.pose.orientation.w = 1.0;
            label.scale.z         = 0.10f;
            label.text            = "F" + std::to_string(e.id)
                                  + "\n" + std::string(1, to_cstr(e.status)[0]);
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
            array.markers.push_back(label);
        }

        markers_pub_->publish(array);
    }

    // ── members ───────────────────────────────────────────────────────────────
    std::mutex                              mutex_;
    std::map<int32_t, FrontierEntry>        entries_;   // ordered by id (insertion time)
    int32_t                                 next_id_{0};

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr         frontiers_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr     markers_pub_;
    rclcpp::Service<shelfbot::srv::GetNextFrontier>::SharedPtr             svc_next_;
    rclcpp::Service<shelfbot::srv::UpdateFrontierStatus>::SharedPtr        svc_update_;
    rclcpp::Service<shelfbot::srv::GetQueueSummary>::SharedPtr             svc_summary_;
    rclcpp::TimerBase::SharedPtr                                           marker_timer_;
    rclcpp::TimerBase::SharedPtr                                           requeue_timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierQueueNode>());
    rclcpp::shutdown();
    return 0;
}
