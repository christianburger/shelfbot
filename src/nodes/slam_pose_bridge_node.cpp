// slam_pose_bridge_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const std::array<double, 36> SLAM_POSE_COVARIANCE = {
    0.10, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.10, 0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.15
};

class SlamPoseBridgeNode : public rclcpp::Node
{
public:
    SlamPoseBridgeNode()
        : Node("slam_pose_bridge_node")
    {
        declare_parameter<double>("publish_rate_hz", 5.0);
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_footprint");
        declare_parameter<double>("tf_timeout_s", 0.3);

        double rate_hz = get_parameter("publish_rate_hz").as_double();
        map_frame_ = get_parameter("map_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        tf_timeout_s_ = get_parameter("tf_timeout_s").as_double();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/slam_pose", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&SlamPoseBridgeNode::publish_pose, this));

        RCLCPP_INFO(get_logger(), "Bridge publishing odom→%s on /slam_pose at %.1f Hz",
                    base_frame_.c_str(), rate_hz);
    }

private:
    void publish_pose()
    {
        geometry_msgs::msg::TransformStamped map_to_base, map_to_odom;
        try {
            // Get SLAM's map→base_footprint
            map_to_base = tf_buffer_->lookupTransform(
                map_frame_, base_frame_,
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(tf_timeout_s_));

            // Get SLAM's map→odom
            map_to_odom = tf_buffer_->lookupTransform(
                map_frame_, odom_frame_,
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(tf_timeout_s_));
        } catch (const tf2::TransformException & ex) {
            return;  // not ready yet
        }

        tf2::Transform tf_map_to_base, tf_map_to_odom;
        tf2::fromMsg(map_to_base.transform, tf_map_to_base);
        tf2::fromMsg(map_to_odom.transform, tf_map_to_odom);

        // Compute odom→base_footprint
        tf2::Transform tf_odom_to_base = tf_map_to_odom.inverse() * tf_map_to_base;

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = map_to_base.header.stamp;
        msg.header.frame_id = odom_frame_;   // publish in odom frame
        tf2::toMsg(tf_odom_to_base, msg.pose.pose);
        msg.pose.covariance = SLAM_POSE_COVARIANCE;

        pose_pub_->publish(msg);
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string map_frame_, odom_frame_, base_frame_;
    double tf_timeout_s_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamPoseBridgeNode>());
    rclcpp::shutdown();
    return 0;
}