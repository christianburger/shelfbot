// lidar_relay_node.cpp
//
// Subscribes  : /shelfbot_firmware/laser_scan  (sensor_msgs/LaserScan, RELIABLE)
//               Complete revolution published by the firmware once per full
//               lidar rotation (~0.36 Hz observed, LYDSTO LDS02RR).
//
// Publishes   : /scan  (sensor_msgs/LaserScan, BEST_EFFORT / SensorDataQoS)
//               Message with frame_id rewritten to 'laser_link' and stamp
//               set to (now - stamp_offset_ms) in the laptop clock domain.
//
// WHY RE-STAMP WITH AN OFFSET:
//
//   Problem 1 — clock domain mismatch:
//     The firmware timestamps scans with its SNTP-synced clock which drifts
//     relative to the laptop clock. slam_toolbox's MessageFilter looks up
//     odom → laser_link at the scan's timestamp using the laptop TF buffer.
//     If firmware clock is ahead of laptop clock, the lookup requests a TF
//     from the future — tf2 refuses to extrapolate — lookup fails — the
//     MessageFilter queue (size=1) fills up — every subsequent scan is
//     dropped with "discarding message because the queue is full".
//
//   Problem 2 — re-stamp to now() is also insufficient:
//     Setting stamp = this->now() moves the timestamp into the laptop clock
//     domain, but the odom→base_footprint TF publishes at only 2 Hz (the
//     hardware interface update_rate). Between two TF publications there is
//     a 500ms gap. If the scan is re-stamped to "now" and "now" is up to
//     500ms newer than the latest TF entry in slam_toolbox's buffer, tf2
//     again refuses to extrapolate forward — same failure mode.
//
//   Solution — stamp behind now():
//     Re-stamp the scan to (now - STAMP_OFFSET_MS). The offset must be:
//       > TF publish period (500ms at 2 Hz hardware update_rate)
//     so that the scan timestamp always falls BETWEEN two existing TF entries,
//     never past the latest one. 200ms provides 40% headroom over the 500ms
//     TF period — use 600ms to comfortably clear it.
//
//     Nav2 costmaps and slam_toolbox both tolerate a fixed latency offset;
//     they do not require the scan stamp to match wall-clock exactly.
//     The offset is constant so it does not affect SLAM quality.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

static constexpr const char* LOG_TAG = "LidarRelay";

// Offset subtracted from now() when re-stamping each scan.
// Must exceed the TF publish period (hardware update_rate=2Hz → 500ms).
// 600ms provides comfortable margin; increase if "queue is full" reappears.
static constexpr int64_t STAMP_OFFSET_NS = 600'000'000;  // 600 ms in nanoseconds

class LidarRelayNode : public rclcpp::Node {
public:
    LidarRelayNode() : Node("lidar_relay_node") {

        declare_parameter<std::string>("frame_id", "laser_link");
        frame_id_ = get_parameter("frame_id").as_string();

        // ── Subscriber ────────────────────────────────────────────────────
        // Firmware publishes RELIABLE; we must match to avoid silent QoS drop.
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/shelfbot_firmware/laser_scan",
            sub_qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                on_scan(std::move(msg));
            });

        // ── Publisher ─────────────────────────────────────────────────────
        // Nav2 and slam_toolbox subscribe to /scan with BEST_EFFORT.
        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(),
            "[%s] relay /shelfbot_firmware/laser_scan (RELIABLE) "
            "→ /scan (BEST_EFFORT)  frame_id='%s'  stamp_offset=%ldms",
            LOG_TAG, frame_id_.c_str(), STAMP_OFFSET_NS / 1'000'000);
    }

private:
    void on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // Re-stamp: laptop clock minus offset.
        // Guarantees scan.stamp is always in the past relative to the latest
        // TF entry in slam_toolbox's buffer, so every tf2 lookup succeeds.
        const rclcpp::Time restamped = this->now() - rclcpp::Duration(0, STAMP_OFFSET_NS);

        if (!first_received_) {
            first_received_ = true;
            RCLCPP_INFO(get_logger(),
                "[%s] First scan: %zu ranges  "
                "angle_min=%.3f  angle_max=%.3f  "
                "range_min=%.3f m  range_max=%.3f m  "
                "firmware_stamp=%d.%09d  restamped=%.3f  "
                "original frame_id='%s'",
                LOG_TAG,
                msg->ranges.size(),
                msg->angle_min, msg->angle_max,
                msg->range_min, msg->range_max,
                msg->header.stamp.sec, msg->header.stamp.nanosec,
                restamped.seconds(),
                msg->header.frame_id.c_str());
        }

        msg->header.stamp    = restamped;
        msg->header.frame_id = frame_id_;

        pub_->publish(std::move(*msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    pub_;
    std::string frame_id_;
    bool        first_received_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRelayNode>());
    rclcpp::shutdown();
    return 0;
}
