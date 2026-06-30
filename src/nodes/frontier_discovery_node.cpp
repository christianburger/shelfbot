// ─────────────────────────────────────────────────────────────────────────────
// frontier_discovery_node.cpp
//
// INDEPENDENT node.  Knows only about /map.
// Converts OccupancyGrid → frontier centroid PoseArray using BFS connected-
// component labelling — no OpenCV dependency.
//
// ─── Interfaces ───────────────────────────────────────────────────────────────
// Subscribe  /map                       nav_msgs/OccupancyGrid  TRANSIENT_LOCAL
// Publish    /frontiers                 geometry_msgs/PoseArray  (map frame)
// Publish    /frontiers/markers         visualization_msgs/MarkerArray
// Service    ~/get_frontiers            shelfbot/srv/GetFrontiers
// ─────────────────────────────────────────────────────────────────────────────
#include <queue>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>

#include <shelfbot/srv/get_frontiers.hpp>

namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Pure algorithmic helpers (no ROS2 types)
// ─────────────────────────────────────────────────────────────────────────────

// Returns true if cell (r,c) is free (==0) and has at least one unknown
// neighbour (==-1) in the 8-connected neighbourhood.
bool is_frontier_cell(const std::vector<int8_t>& data,
                      int r, int c, int width, int height) {
    if (data[r * width + c] != 0) return false;
    for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            int nr = r + dr, nc = c + dc;
            if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;
            if (data[nr * width + nc] == -1) return true;
        }
    }
    return false;
}

// BFS flood-fill from seed (sr, sc) over the frontier mask.
// Returns the list of (row, col) cells belonging to the component.
std::vector<std::pair<int,int>> bfs_component(
        const std::vector<bool>& frontier_mask,
        std::vector<bool>& visited,
        int sr, int sc, int width, int height) {
    std::vector<std::pair<int,int>> component;
    std::queue<std::pair<int,int>> q;
    q.push({sr, sc});
    visited[sr * width + sc] = true;

    while (!q.empty()) {
        auto [r, c] = q.front(); q.pop();
        component.emplace_back(r, c);
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                if (dr == 0 && dc == 0) continue;
                int nr = r + dr, nc = c + dc;
                if (nr < 0 || nr >= height || nc < 0 || nc >= width) continue;
                int idx = nr * width + nc;
                if (!visited[idx] && frontier_mask[idx]) {
                    visited[idx] = true;
                    q.push({nr, nc});
                }
            }
        }
    }
    return component;
}

// Full extraction: return one (cx, cy) centroid per region of size ≥ min_size.
std::vector<std::pair<double,double>> extract_frontier_centroids(
        const nav_msgs::msg::OccupancyGrid& occ,
        int min_size) {
    const int W   = static_cast<int>(occ.info.width);
    const int H   = static_cast<int>(occ.info.height);
    const double res = occ.info.resolution;
    const double ox  = occ.info.origin.position.x;
    const double oy  = occ.info.origin.position.y;

    // Build frontier mask
    std::vector<bool> frontier_mask(W * H, false);
    for (int r = 0; r < H; ++r)
        for (int c = 0; c < W; ++c)
            if (is_frontier_cell(occ.data, r, c, W, H))
                frontier_mask[r * W + c] = true;

    // BFS connected components → centroids
    std::vector<bool> visited(W * H, false);
    std::vector<std::pair<double,double>> centroids;

    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            int idx = r * W + c;
            if (!frontier_mask[idx] || visited[idx]) continue;

            auto component = bfs_component(frontier_mask, visited, r, c, W, H);
            if (static_cast<int>(component.size()) < min_size) continue;

            double sum_x = 0.0, sum_y = 0.0;
            for (auto [cr, cc] : component) {
                sum_x += static_cast<double>(cc) * res + ox + res / 2.0;
                sum_y += static_cast<double>(cr) * res + oy + res / 2.0;
            }
            centroids.emplace_back(sum_x / component.size(),
                                   sum_y / component.size());
        }
    }
    return centroids;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
class FrontierDiscoveryNode : public rclcpp::Node {
public:
    explicit FrontierDiscoveryNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("frontier_discovery", opts)
    {
        declare_parameter("min_frontier_size", 5);
        declare_parameter("publish_hz",        2.0);

        // TRANSIENT_LOCAL: receive slam_toolbox's last /map even if subscribed late
        rclcpp::QoS map_qos(1);
        map_qos.transient_local().reliable();

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                latest_map_ = std::move(msg);
            });

        frontiers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/frontiers", 10);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontiers/markers", 10);

        service_ = create_service<shelfbot::srv::GetFrontiers>(
            "~/get_frontiers",
            [this](const shelfbot::srv::GetFrontiers::Request::SharedPtr req,
                         shelfbot::srv::GetFrontiers::Response::SharedPtr res) {
                handle_get_frontiers(req, res);
            });

        const double period = 1.0 / get_parameter("publish_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            [this] { publish_tick(); });

        RCLCPP_INFO(get_logger(), "FrontierDiscovery ready — waiting for /map.");
    }

private:
    // ── service ───────────────────────────────────────────────────────────────
    void handle_get_frontiers(
            const shelfbot::srv::GetFrontiers::Request::SharedPtr  req,
                  shelfbot::srv::GetFrontiers::Response::SharedPtr res) {
        if (!latest_map_) { res->count = 0; return; }
        const int min_sz = (req->min_size > 0)
            ? req->min_size
            : get_parameter("min_frontier_size").as_int();

        auto centroids = extract_frontier_centroids(*latest_map_, min_sz);
        res->poses = build_pose_array(centroids);
        res->count = static_cast<int32_t>(centroids.size());
    }

    // ── timer ─────────────────────────────────────────────────────────────────
    void publish_tick() {
        if (!latest_map_) return;
        const int min_sz = get_parameter("min_frontier_size").as_int();
        auto centroids   = extract_frontier_centroids(*latest_map_, min_sz);

        frontiers_pub_->publish(build_pose_array(centroids));
        markers_pub_->publish(build_markers(centroids));

        RCLCPP_DEBUG(get_logger(), "Published %zu frontier centroids.",
                     centroids.size());
    }

    // ── builders ──────────────────────────────────────────────────────────────
    std_msgs::msg::Header map_header() const {
        std_msgs::msg::Header h;
        h.frame_id = "map";
        h.stamp    = now();
        return h;
    }

    geometry_msgs::msg::PoseArray build_pose_array(
            const std::vector<std::pair<double,double>>& centroids) const {
        geometry_msgs::msg::PoseArray pa;
        pa.header = map_header();
        for (auto [cx, cy] : centroids) {
            geometry_msgs::msg::Pose p;
            p.position.x    = cx;
            p.position.y    = cy;
            p.orientation.w = 1.0;
            pa.poses.push_back(p);
        }
        return pa;
    }

    visualization_msgs::msg::MarkerArray build_markers(
            const std::vector<std::pair<double,double>>& centroids) const {
        visualization_msgs::msg::MarkerArray array;
        auto hdr = map_header();

        // Clear stale markers from the previous publish
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        clear.header = hdr;
        array.markers.push_back(clear);

        int id = 0;
        for (auto [cx, cy] : centroids) {
            visualization_msgs::msg::Marker m;
            m.header      = hdr;
            m.ns          = "frontiers";
            m.id          = id++;
            m.type        = visualization_msgs::msg::Marker::CYLINDER;
            m.action      = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x    = cx;
            m.pose.position.y    = cy;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = 0.18;
            m.scale.z = 0.04;
            m.color.r = 0.0f; m.color.g = 0.85f;
            m.color.b = 1.0f; m.color.a = 0.80f;
            array.markers.push_back(m);
        }
        return array;
    }

    // ── members ───────────────────────────────────────────────────────────────
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    frontiers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Service<shelfbot::srv::GetFrontiers>::SharedPtr        service_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDiscoveryNode>());
    rclcpp::shutdown();
    return 0;
}
