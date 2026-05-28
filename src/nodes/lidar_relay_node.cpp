// Subscribes  : /shelfbot_firmware/lidar_scan  (std_msgs/Float32MultiArray)
// Publishes   : /scan                          (sensor_msgs/LaserScan)
//
// Packet layout – 30 floats (must match LIDAR_MSG_SIZE in shelfbot.hpp):
//  [0]      start_angle_deg
//  [1]      end_angle_deg
//  [2]      min_distance_angle_deg
//  [3]      min distance_mm
//  [4]      packet speed (rpm field from LYDSTO)
//  [5..16]  distances_mm[0..11]
//  [17..28] confidences[0..11]
//  [29]     valid flag  (1.0 = valid, 0.0 = invalid)
//
// Algorithm:
//   Each incoming packet covers a ~30° arc (12 points over start→end angle).
//   Points are binned into a 360-bucket array (1° per bucket); the closest
//   return per bucket is kept.  A wall-timer publishes the accumulated array
//   as a LaserScan at `publish_hz` (default 5 Hz).  Buckets older than
//   2 × publish period are flushed to infinity so Nav2 never sees ghosts.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <chrono>

static constexpr int   NUM_POINTS       = 12;
static constexpr float RANGE_MIN_M      = 0.05f;   // 5 cm  – LYDSTO minimum
static constexpr float RANGE_MAX_M      = 8.0f;    // 8 m   – LYDSTO maximum
static constexpr int   BUCKET_COUNT     = 360;     // 1° resolution
static constexpr double DEFAULT_HZ      = 5.0;
static constexpr int   DEFAULT_MIN_CONF = 10;

class LidarRelayNode : public rclcpp::Node {
public:
    LidarRelayNode() : Node("lidar_relay_node") {
        this->declare_parameter<std::string>("frame_id",      "laser_link");
        this->declare_parameter<double>     ("publish_hz",     DEFAULT_HZ);
        this->declare_parameter<int>        ("min_confidence", DEFAULT_MIN_CONF);

        frame_id_       = this->get_parameter("frame_id").as_string();
        publish_hz_     = this->get_parameter("publish_hz").as_double();
        min_confidence_ = this->get_parameter("min_confidence").as_int();

        bucket_ranges_.assign(BUCKET_COUNT, std::numeric_limits<float>::infinity());
        bucket_stamps_.assign(BUCKET_COUNT, rclcpp::Time(0, 0, RCL_ROS_TIME));

        sub_ = this->create_subscription<Float32MA>(
            "/shelfbot_firmware/lidar_scan",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarRelayNode::on_packet, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS());

        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_hz_));
        publish_timer_ = this->create_wall_timer(
            period_ns, std::bind(&LidarRelayNode::publish_scan, this));

        RCLCPP_INFO(get_logger(),
            "lidar_relay_node ready  frame_id=%s  publish_hz=%.1f  min_confidence=%d",
            frame_id_.c_str(), publish_hz_, min_confidence_);
    }

private:
    using Float32MA = std_msgs::msg::Float32MultiArray;
    using LaserScan = sensor_msgs::msg::LaserScan;

    // ------------------------------------------------------------------
    void on_packet(const Float32MA::SharedPtr msg) {
        if (static_cast<int>(msg->data.size()) < 30) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "lidar_relay: undersized packet (%zu floats, need 30)",
                msg->data.size());
            return;
        }

        // Firmware invalid flag
        if (msg->data[29] < 0.5f) return;

        const float start_deg = msg->data[0];
        const float end_deg   = msg->data[1];

        float span = end_deg - start_deg;
        if (span < 0.0f) span += 360.0f;
        if (span < 0.1f) return;   // degenerate

        const rclcpp::Time now = this->now();

        for (int i = 0; i < NUM_POINTS; ++i) {
            const float dist_mm = msg->data[5  + i];
            const float conf    = msg->data[17 + i];

            if (static_cast<int>(conf) < min_confidence_) continue;
            if (dist_mm < 50.0f || dist_mm > 8000.0f)    continue;

            // Angle of this point within the arc
            const float angle_deg = start_deg
                + span * static_cast<float>(i)
                       / static_cast<float>(NUM_POINTS - 1);

            // Normalise to [0, 360)
            float norm = std::fmod(angle_deg, 360.0f);
            if (norm < 0.0f) norm += 360.0f;

            const int   bucket = static_cast<int>(norm) % BUCKET_COUNT;
            const float dist_m = dist_mm / 1000.0f;

            // Keep the closest return (nearest obstacle wins)
            if (dist_m < bucket_ranges_[bucket]) {
                bucket_ranges_[bucket] = dist_m;
                bucket_stamps_[bucket] = now;
            }
        }

        ++packets_since_publish_;
    }

    // ------------------------------------------------------------------
    void publish_scan() {
        if (packets_since_publish_ == 0) {
            RCLCPP_DEBUG(get_logger(),
                "lidar_relay: no new packets — publishing stale/empty scan");
        }

        const rclcpp::Time now       = this->now();
        const double       max_age_s = 2.0 / publish_hz_;

        LaserScan scan;
        scan.header.stamp    = now;
        scan.header.frame_id = frame_id_;

        // Full 360°, 1° resolution, angles in radians increasing CCW
        scan.angle_min       = 0.0f;
        scan.angle_max       = 2.0f * static_cast<float>(M_PI);
        scan.angle_increment = 2.0f * static_cast<float>(M_PI)
                               / static_cast<float>(BUCKET_COUNT);
        scan.time_increment  = 0.0f;
        scan.scan_time       = static_cast<float>(1.0 / publish_hz_);
        scan.range_min       = RANGE_MIN_M;
        scan.range_max       = RANGE_MAX_M;
        scan.ranges.resize(BUCKET_COUNT);

        for (int b = 0; b < BUCKET_COUNT; ++b) {
            const double age_s = (now - bucket_stamps_[b]).seconds();
            if (bucket_ranges_[b] < std::numeric_limits<float>::infinity()
                && age_s <= max_age_s) {
                scan.ranges[b] = bucket_ranges_[b];
            } else {
                scan.ranges[b]    = std::numeric_limits<float>::infinity();
                bucket_ranges_[b] = std::numeric_limits<float>::infinity();  // reset stale
            }
        }

        pub_->publish(scan);
        RCLCPP_DEBUG(get_logger(),
            "lidar_relay: published /scan  packets_accumulated=%d",
            packets_since_publish_);
        packets_since_publish_ = 0;
    }

    // ------------------------------------------------------------------
    rclcpp::Subscription<Float32MA>::SharedPtr sub_;
    rclcpp::Publisher<LaserScan>::SharedPtr    pub_;
    rclcpp::TimerBase::SharedPtr               publish_timer_;

    std::string frame_id_;
    double      publish_hz_          {DEFAULT_HZ};
    int         min_confidence_      {DEFAULT_MIN_CONF};
    int         packets_since_publish_{0};

    std::vector<float>         bucket_ranges_;
    std::vector<rclcpp::Time>  bucket_stamps_;
};

// --------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRelayNode>());
    rclcpp::shutdown();
    return 0;
}
