// lidar_relay_node.cpp
//
// Subscribes  : /shelfbot_firmware/laser_scan  (sensor_msgs/LaserScan)
// Publishes   : /scan  (sensor_msgs/LaserScan) – full 360° accumulation

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>
#include <vector>
#include <chrono>

static constexpr const char* LOG_TAG = "LidarRelay";
static constexpr int    BUCKET_COUNT  = 360;
static constexpr double DEFAULT_HZ    = 10.0;
static constexpr float  TWO_PI        = 2.0f * static_cast<float>(M_PI);
static constexpr float  DEG_TO_RAD    = static_cast<float>(M_PI) / 180.0f;

class LidarRelayNode : public rclcpp::Node {
public:
    LidarRelayNode() : Node("lidar_relay_node") {

        declare_parameter<std::string>("input_topic",  "/shelfbot_firmware/laser_scan");
        declare_parameter<std::string>("output_topic", "/scan");
        declare_parameter<std::string>("frame_id",     "lidar_frame");
        declare_parameter<double>     ("publish_hz",    DEFAULT_HZ);
        declare_parameter<double>     ("range_min_override", -1.0);
        declare_parameter<double>     ("range_max_override", -1.0);
        declare_parameter<double>     ("expiry_periods", 3.0);
        declare_parameter<std::string>("qos_reliability", "reliable");

        input_topic_        = get_parameter("input_topic").as_string();
        output_topic_       = get_parameter("output_topic").as_string();
        frame_id_           = get_parameter("frame_id").as_string();
        publish_hz_         = get_parameter("publish_hz").as_double();
        range_min_override_ = static_cast<float>(get_parameter("range_min_override").as_double());
        range_max_override_ = static_cast<float>(get_parameter("range_max_override").as_double());
        expiry_periods_     = get_parameter("expiry_periods").as_double();
        qos_reliability_    = get_parameter("qos_reliability").as_string();

        if (publish_hz_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "[%s] Invalid publish_hz=%.2f, falling back to %.2f",
                        LOG_TAG, publish_hz_, DEFAULT_HZ);
            publish_hz_ = DEFAULT_HZ;
        }

        bucket_ranges_.assign(BUCKET_COUNT, std::numeric_limits<float>::infinity());
        bucket_stamps_.assign(BUCKET_COUNT, rclcpp::Time(0, 0, RCL_ROS_TIME));

        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        if (qos_reliability_ == "best_effort") {
            qos_profile.best_effort();
            RCLCPP_INFO(get_logger(), "[%s] Subscriber QoS: BEST_EFFORT", LOG_TAG);
        } else {
            qos_profile.reliable();
            RCLCPP_INFO(get_logger(), "[%s] Subscriber QoS: RELIABLE", LOG_TAG);
        }

        sub_ = create_subscription<LaserScan>(
            input_topic_, qos_profile,
            std::bind(&LidarRelayNode::on_scan, this, std::placeholders::_1));

        pub_ = create_publisher<LaserScan>(output_topic_, rclcpp::SensorDataQoS());

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_hz_));
        publish_timer_ = create_wall_timer(
            period_ns, std::bind(&LidarRelayNode::publish_scan, this));

        diag_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&LidarRelayNode::print_diagnostics, this));

        RCLCPP_INFO(get_logger(), "[%s] Initialised. Subscribing to '%s', publishing to '%s' (frame_id=%s, %.1f Hz, expiry=%.1f periods)",
                    LOG_TAG, input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), publish_hz_, expiry_periods_);
    }

private:
    using LaserScan = sensor_msgs::msg::LaserScan;

    void on_scan(const LaserScan::SharedPtr msg) {
        if (total_packets_ == 0) {
            RCLCPP_INFO(get_logger(), "[%s] First packet: %zu ranges | angle_min=%.3f rad (%.1f°) angle_max=%.3f rad (%.1f°) | inc=%.4f | range_min=%.3f range_max=%.3f | frame='%s'",
                        LOG_TAG,
                        msg->ranges.size(),
                        msg->angle_min, msg->angle_min * 180.0 / M_PI,
                        msg->angle_max, msg->angle_max * 180.0 / M_PI,
                        msg->angle_increment,
                        msg->range_min, msg->range_max,
                        msg->header.frame_id.c_str());
        }

        if (msg->ranges.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[%s] Received packet with empty ranges – skipping", LOG_TAG);
            return;
        }

        // Only reject if increment is exactly zero (no sensible scan possible)
        if (msg->angle_increment == 0.0f) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[%s] Zero angle_increment – skipping packet", LOG_TAG);
            return;
        }

        if (!firmware_limits_latched_) {
            firmware_range_min_ = msg->range_min;
            firmware_range_max_ = msg->range_max;
            firmware_limits_latched_ = true;
            RCLCPP_INFO(get_logger(), "[%s] Firmware range limits latched: min=%.3f m, max=%.3f m",
                        LOG_TAG, firmware_range_min_, firmware_range_max_);
        }

        const float r_min = msg->range_min;
        const float r_max = msg->range_max;
        const rclcpp::Time stamp = this->now();

        int inserted = 0;
        int discarded_range = 0;
        int discarded_nonfinite = 0;
        int not_inserted_closer = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const float r = msg->ranges[i];

            if (!std::isfinite(r)) {
                ++discarded_nonfinite;
                continue;
            }
            if (r < r_min || r > r_max) {
                ++discarded_range;
                continue;
            }

            // Angle calculation works for both positive and negative increment
            float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            angle = std::fmod(angle, TWO_PI);
            if (angle < 0.0f) angle += TWO_PI;

            int bucket = static_cast<int>(angle / DEG_TO_RAD) % BUCKET_COUNT;
            if (bucket < 0) bucket += BUCKET_COUNT;

            // Closest return wins
            if (r < bucket_ranges_[bucket]) {
                bucket_ranges_[bucket] = r;
                bucket_stamps_[bucket] = stamp;
                ++inserted;
            } else {
                ++not_inserted_closer;
            }
        }

        total_packets_++;
        packets_this_cycle_++;

        RCLCPP_DEBUG(get_logger(),
            "[%s] Packet %lu: %zu ranges → %d inserted, %d not closer, %d non‑finite, %d out‑of‑range | sector (%.1f°–%.1f°)",
            LOG_TAG, total_packets_, msg->ranges.size(),
            inserted, not_inserted_closer, discarded_nonfinite, discarded_range,
            msg->angle_min * 180.0 / M_PI, msg->angle_max * 180.0 / M_PI);

        if (inserted == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "[%s] Packet %lu: No new insertions (all ranges were either invalid or not closer). "
                "Valid but not closer: %d, non‑finite: %d, out‑of‑range: %d",
                LOG_TAG, total_packets_, not_inserted_closer, discarded_nonfinite, discarded_range);
        }
    }

    void publish_scan() {
        const rclcpp::Time now = this->now();
        const double max_age_s = expiry_periods_ / publish_hz_;

        const float r_min = (range_min_override_ > 0.0f) ? range_min_override_ : firmware_range_min_;
        const float r_max = (range_max_override_ > 0.0f) ? range_max_override_ : firmware_range_max_;

        LaserScan out;
        out.header.stamp = now;
        out.header.frame_id = frame_id_;

        out.angle_min       = 0.0f;
        out.angle_max       = static_cast<float>(BUCKET_COUNT - 1) * DEG_TO_RAD;
        out.angle_increment = DEG_TO_RAD;
        out.time_increment  = 0.0f;
        out.scan_time       = static_cast<float>(1.0 / publish_hz_);
        out.range_min       = r_min;
        out.range_max       = r_max;
        out.ranges.resize(BUCKET_COUNT);

        int populated = 0;
        for (int b = 0; b < BUCKET_COUNT; ++b) {
            const float r = bucket_ranges_[b];
            bool valid = std::isfinite(r) && r >= r_min && r <= r_max;
            if (valid && bucket_stamps_[b].nanoseconds() > 0) {
                if ((now - bucket_stamps_[b]).seconds() > max_age_s) {
                    bucket_ranges_[b] = std::numeric_limits<float>::infinity();
                    valid = false;
                }
            }
            out.ranges[b] = valid ? r : std::numeric_limits<float>::infinity();
            if (valid) ++populated;
        }

        pub_->publish(out);

        if (packets_this_cycle_ == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[%s] PUBLISH: No packets received in this cycle! QoS=%s, total packets ever=%lu",
                LOG_TAG, qos_reliability_.c_str(), total_packets_);
        }

        RCLCPP_DEBUG(get_logger(),
            "[%s] PUBLISH: %d/360 buckets populated (%d packets this cycle)",
            LOG_TAG, populated, packets_this_cycle_);

        packets_this_cycle_ = 0;
    }

    void print_diagnostics() {
        const rclcpp::Time now = this->now();
        int populated = 0;
        int first_bucket = -1, last_bucket = -1;
        const double max_age_s = expiry_periods_ / publish_hz_;

        for (int b = 0; b < BUCKET_COUNT; ++b) {
            if (std::isfinite(bucket_ranges_[b]) &&
                bucket_stamps_[b].nanoseconds() > 0 &&
                (now - bucket_stamps_[b]).seconds() <= max_age_s) {
                ++populated;
                if (first_bucket < 0) first_bucket = b;
                last_bucket = b;
            }
        }

        if (populated > 0) {
            RCLCPP_INFO(get_logger(),
                "[%s] DIAG: %d/360 buckets populated (%d°–%d°), total received packets: %lu",
                LOG_TAG, populated,
                static_cast<int>(first_bucket * DEG_TO_RAD * 180.0 / M_PI),
                static_cast<int>(last_bucket * DEG_TO_RAD * 180.0 / M_PI),
                total_packets_);
        } else {
            RCLCPP_WARN(get_logger(),
                "[%s] DIAG: 0 buckets populated. Total received packets: %lu. Check firmware or QoS (current=%s).",
                LOG_TAG, total_packets_, qos_reliability_.c_str());
        }
    }

    rclcpp::Subscription<LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<LaserScan>::SharedPtr    pub_;
    rclcpp::TimerBase::SharedPtr               publish_timer_;
    rclcpp::TimerBase::SharedPtr               diag_timer_;

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    double      publish_hz_         {DEFAULT_HZ};
    float       range_min_override_ {-1.0f};
    float       range_max_override_ {-1.0f};
    double      expiry_periods_     {3.0};
    std::string qos_reliability_    {"reliable"};

    std::vector<float>        bucket_ranges_;
    std::vector<rclcpp::Time> bucket_stamps_;

    bool  firmware_limits_latched_ {false};
    float firmware_range_min_      {0.02f};
    float firmware_range_max_      {12.0f};

    uint64_t total_packets_          {0};
    int      packets_this_cycle_     {0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRelayNode>());
    rclcpp::shutdown();
    return 0;
}