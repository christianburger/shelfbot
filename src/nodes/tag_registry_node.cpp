
// ─────────────────────────────────────────────────────────────────────────────
// tag_registry_node.cpp
//
// INDEPENDENT node.  No frontier or navigation knowledge.
// Subscribes to /tag_detections (apriltag_msgs/AprilTagDetectionArray),
// transforms each detection into the map frame via TF, and stores the result
// in a thread-safe registry.
//
// Averaging strategy
//   When the same tag_id is seen again, the stored pose is updated with an
//   exponential moving average (alpha = 0.1) weighted toward the new
//   measurement, giving a noise-smoothed estimate over time.
//
// ─── Interfaces ───────────────────────────────────────────────────────────────
// Subscribe  /tag_detections           apriltag_msgs/AprilTagDetectionArray
// Publish    /tag_registry/markers     visualization_msgs/MarkerArray
// Service    ~/check_found             shelfbot/srv/CheckTagsFound
// ─────────────────────────────────────────────────────────────────────────────
#include <map>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <shelfbot/srv/check_tags_found.hpp>

// ─────────────────────────────────────────────────────────────────────────────
struct TagEntry {
    int32_t                         id;
    geometry_msgs::msg::PoseStamped pose_in_map;   // running average
    int32_t                         observation_count{0};
};

// ─────────────────────────────────────────────────────────────────────────────
class TagRegistryNode : public rclcpp::Node {
public:
    explicit TagRegistryNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("tag_registry", opts)
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_)
    {
        declare_parameter("map_frame",         std::string("map"));
        declare_parameter("camera_frame",      std::string("camera_link_optical_frame"));
        declare_parameter("pose_avg_alpha",    0.10);  // EMA weight for new measurements
        declare_parameter("publish_hz",        2.0);

        // BEST_EFFORT matches the default camera / detector QoS on embedded hardware
        rclcpp::QoS det_qos(10);
        det_qos.best_effort();

        detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/tag_detections", det_qos,
            [this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
                on_detections(msg);
            });

        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/tag_registry/markers", 10);

        svc_check_ = create_service<shelfbot::srv::CheckTagsFound>(
            "~/check_found",
            [this](const shelfbot::srv::CheckTagsFound::Request::SharedPtr  req,
                         shelfbot::srv::CheckTagsFound::Response::SharedPtr res) {
                handle_check(req, res);
            });

        const double period = 1.0 / get_parameter("publish_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            [this] { publish_markers(); });

        RCLCPP_INFO(get_logger(), "TagRegistry ready.");
    }

private:
    // ── detection callback ────────────────────────────────────────────────────
    void on_detections(
            const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        const std::string map_frame    = get_parameter("map_frame").as_string();
        const std::string camera_frame = get_parameter("camera_frame").as_string();
        const double      alpha        = get_parameter("pose_avg_alpha").as_double();

        for (const auto& det : msg->detections) {
            // apriltag_msgs (christianrauch) — the package this node actually
            // depends on — does NOT embed a pose in AprilTagDetection; only
            // family/id/hamming/decision_margin/centre/corners/homography.
            // apriltag_ros instead publishes each tag's pose directly on
            // /tf, with child_frame_id "tag<family>:<id>" (e.g. "tag36h11:0").
            // Look that transform up relative to camera_frame here, exactly
            // the pose ps_cam below always represented, then hand it to the
            // existing camera_frame → map_frame transform unchanged.
            const std::string tag_frame =
                "tag" + det.family + ":" + std::to_string(det.id);

            geometry_msgs::msg::PoseStamped ps_cam;
            try {
                const auto tf_cam = tf_buffer_.lookupTransform(
                    camera_frame, tag_frame, tf2::TimePointZero,
                    tf2::durationFromSec(0.15));
                ps_cam.header.frame_id    = camera_frame;
                ps_cam.header.stamp       = tf_cam.header.stamp;
                ps_cam.pose.position.x    = tf_cam.transform.translation.x;
                ps_cam.pose.position.y    = tf_cam.transform.translation.y;
                ps_cam.pose.position.z    = tf_cam.transform.translation.z;
                ps_cam.pose.orientation   = tf_cam.transform.rotation;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TF for tag %d (%s): %s", det.id, tag_frame.c_str(), ex.what());
                continue;
            }

            // Transform to map frame
            geometry_msgs::msg::PoseStamped ps_map;
            try {
                tf_buffer_.transform(ps_cam, ps_map, map_frame,
                    tf2::durationFromSec(0.15));
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TF for tag %d: %s", det.id, ex.what());
                continue;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            auto it = registry_.find(det.id);
            if (it == registry_.end()) {
                // First sighting — store directly
                TagEntry entry;
                entry.id                  = det.id;
                entry.pose_in_map         = ps_map;
                entry.observation_count   = 1;
                registry_.emplace(det.id, entry);
                RCLCPP_INFO(get_logger(),
                    "Tag %d first seen at map (%.3f, %.3f).",
                    det.id,
                    ps_map.pose.position.x,
                    ps_map.pose.position.y);
            } else {
                // Subsequent sighting — EMA update of position
                auto& stored = it->second.pose_in_map.pose.position;
                const auto& meas = ps_map.pose.position;
                stored.x = stored.x * (1.0 - alpha) + meas.x * alpha;
                stored.y = stored.y * (1.0 - alpha) + meas.y * alpha;
                stored.z = stored.z * (1.0 - alpha) + meas.z * alpha;
                it->second.pose_in_map.header.stamp = now();
                it->second.observation_count++;

                RCLCPP_DEBUG(get_logger(),
                    "Tag %d updated (obs=%d): map (%.3f, %.3f).",
                    det.id, it->second.observation_count,
                    stored.x, stored.y);
            }
        }
    }

    // ── service ───────────────────────────────────────────────────────────────
    void handle_check(
            const shelfbot::srv::CheckTagsFound::Request::SharedPtr  req,
                  shelfbot::srv::CheckTagsFound::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);

        // If target_ids is empty, check whether ANY tag has been found
        const bool check_specific = !req->target_ids.empty();
        const auto& targets = req->target_ids;

        bool all_found = true;
        for (int32_t tid : targets) {
            if (registry_.count(tid)) {
                res->found_ids.push_back(tid);
            } else {
                res->missing_ids.push_back(tid);
                all_found = false;
            }
        }

        if (!check_specific) {
            // No specific targets: succeed as soon as any tag is registered
            all_found = !registry_.empty();
        }

        res->all_found = all_found;
    }

    // ── visualisation ─────────────────────────────────────────────────────────
    void publish_markers() {
        visualization_msgs::msg::MarkerArray array;
        std_msgs::msg::Header hdr;
        hdr.frame_id = get_parameter("map_frame").as_string();
        hdr.stamp    = now();

        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        clear.header = hdr;
        array.markers.push_back(clear);

        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [id, entry] : registry_) {
            // Cube marker
            visualization_msgs::msg::Marker cube;
            cube.header   = hdr;
            cube.ns       = "tag_registry";
            cube.id       = id;
            cube.type     = visualization_msgs::msg::Marker::CUBE;
            cube.action   = visualization_msgs::msg::Marker::ADD;
            cube.pose     = entry.pose_in_map.pose;
            cube.scale.x  = cube.scale.y = cube.scale.z = 0.20;
            cube.color.r  = 1.0f; cube.color.g = 0.5f;
            cube.color.b  = 0.0f; cube.color.a = 1.0f;
            array.markers.push_back(cube);

            // Text label
            visualization_msgs::msg::Marker text;
            text.header              = hdr;
            text.ns                  = "tag_registry_labels";
            text.id                  = id;
            text.type                = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action              = visualization_msgs::msg::Marker::ADD;
            text.pose                = entry.pose_in_map.pose;
            text.pose.position.z    += 0.25;
            text.scale.z             = 0.12f;
            text.text                = "TAG " + std::to_string(id)
                                     + " (n=" + std::to_string(entry.observation_count) + ")";
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
            array.markers.push_back(text);
        }

        markers_pub_->publish(array);
    }

    // ── members ───────────────────────────────────────────────────────────────
    std::mutex                          mutex_;
    std::map<int32_t, TagEntry>         registry_;

    tf2_ros::Buffer                     tf_buffer_;
    tf2_ros::TransformListener          tf_listener_;

    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr          markers_pub_;
    rclcpp::Service<shelfbot::srv::CheckTagsFound>::SharedPtr                   svc_check_;
    rclcpp::TimerBase::SharedPtr                                                timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagRegistryNode>());
    rclcpp::shutdown();
    return 0;
}
