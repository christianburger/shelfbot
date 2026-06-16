// lidar_relay_node.cpp
//
// Subscribes  : /shelfbot_firmware/laser_scan  (sensor_msgs/LaserScan, RELIABLE)
//               Complete 360-point revolution published by the firmware once
//               per full lidar rotation (~5 Hz).
//
// Publishes   : /scan  (sensor_msgs/LaserScan, BEST_EFFORT / SensorDataQoS)
//               Identical message with frame_id rewritten to 'laser_link'
//               (the URDF link name broadcast by robot_state_publisher).
//
// There is no accumulation logic here. The firmware is responsible for
// assembling full 360° revolutions before publishing. This node only:
//   1. Receives a complete scan.
//   2. Rewrites frame_id to match the URDF TF frame.
//   3. Re-publishes on /scan with SensorDataQoS so Nav2 and slam_toolbox
//      can subscribe with their default BEST_EFFORT sensor QoS profiles.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

static constexpr const char* LOG_TAG = "LidarRelay";

class LidarRelayNode : public rclcpp::Node {
public:
    LidarRelayNode() : Node("lidar_relay_node") {

        // frame_id to stamp on outgoing /scan messages.
        // Must match the URDF link name (lidar_sensor.xacro: 'laser_link')
        // so that Nav2 costmaps and slam_toolbox can resolve the TF lookup
        // base_footprint → laser_link.
        declare_parameter<std::string>("frame_id", "laser_link");
        frame_id_ = get_parameter("frame_id").as_string();

        // ── Subscriber ────────────────────────────────────────────────────
        // The firmware publishes /shelfbot_firmware/laser_scan with RELIABLE
        // QoS (rclc_publisher_init_default). We must match RELIABLE on our
        // side; a BEST_EFFORT subscriber would be silently incompatible with
        // a RELIABLE publisher in ROS 2 DDS.
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/shelfbot_firmware/laser_scan",
            sub_qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                on_scan(std::move(msg));
            });

        // ── Publisher ─────────────────────────────────────────────────────
        // /scan must be BEST_EFFORT (SensorDataQoS) so that Nav2 costmaps
        // and slam_toolbox — which subscribe with their default sensor QoS
        // (BEST_EFFORT, VOLATILE) — can connect without a QoS mismatch.
        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(),
            "[%s] relay /shelfbot_firmware/laser_scan (RELIABLE) "
            "→ /scan (BEST_EFFORT)  frame_id='%s'",
            LOG_TAG, frame_id_.c_str());
    }

private:
    void on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Log the first message so startup can be verified easily
        if (!first_received_) {
            first_received_ = true;
            RCLCPP_INFO(get_logger(),
                "[%s] First scan received: %zu ranges  "
                "angle_min=%.3f rad  angle_max=%.3f rad  "
                "increment=%.4f rad  range_min=%.3f m  range_max=%.3f m  "
                "original frame_id='%s'",
                LOG_TAG,
                msg->ranges.size(),
                msg->angle_min, msg->angle_max,
                msg->angle_increment,
                msg->range_min, msg->range_max,
                msg->header.frame_id.c_str());
        }

        // Rewrite frame_id — everything else (stamp, ranges, angles) is
        // preserved exactly as the firmware produced it.
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
