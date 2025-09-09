#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/header.hpp"
#include <memory>

// The core apriltag C library headers
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "apriltag_pose.h"
}

class AprilTagDetectorNode : public rclcpp::Node {

public:
    AprilTagDetectorNode();
    ~AprilTagDetectorNode();

private:
    // Member Functions
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    void publishTransforms(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids);
    void publishMarkers(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids, double tag_size);

    // Member Variables
    apriltag_family_t* tf_;
    apriltag_detector_t* td_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Separate subscribers
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // Store latest camera info
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
    bool camera_info_received_ = false;

    // Parameters
    double tag_size_;
    double pose_error_threshold_;
};

AprilTagDetectorNode::AprilTagDetectorNode() : Node("apriltag_detector_node") {
    // Declare and get parameters
    this->declare_parameter<double>("tag_size", 0.16);
    this->declare_parameter<double>("pose_error_threshold", 100.0);
    tag_size_ = this->get_parameter("tag_size").as_double();
    pose_error_threshold_ = this->get_parameter("pose_error_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "Parameters - tag_size: %.3f, pose_error_threshold: %.1f", tag_size_, pose_error_threshold_);

    // Initialize Apriltag detector
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    RCLCPP_INFO(this->get_logger(), "AprilTag detector initialized with tag36h11 family");

    // Publishers
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tag_poses", rclcpp::SensorDataQoS());
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tag_markers", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Publishers created: /tag_poses, /tag_markers");

    // Create separate subscribers
    image_sub_ = image_transport::create_subscription(
        this, "image_raw",
        std::bind(&AprilTagDetectorNode::imageCallback, this, std::placeholders::_1),
        "raw",
        rmw_qos_profile_sensor_data);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 
        rclcpp::SensorDataQoS(),
        std::bind(&AprilTagDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Separate subscribers created for image_raw and camera_info");
}

AprilTagDetectorNode::~AprilTagDetectorNode() {
    RCLCPP_INFO(this->get_logger(), "AprilTag Detector Shutting Down");
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
}

void AprilTagDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    latest_camera_info_ = info_msg;
    if (!camera_info_received_) {
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received - ready to process images");
    }
}

void AprilTagDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    // Check if we have camera info
    if (!camera_info_received_ || !latest_camera_info_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No camera info available yet - skipping image");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Processing image (size: %dx%d, encoding: %s)", 
                 image_msg->width, image_msg->height, image_msg->encoding.c_str());

    cv_bridge::CvImagePtr cv_ptr;
    try {
        // Handle common encodings: prefer MONO8 for tags, but convert from bgr8/rgb8 if needed
        std::string target_encoding = sensor_msgs::image_encodings::MONO8;
        if (image_msg->encoding == sensor_msgs::image_encodings::BGR8 || 
            image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
            target_encoding = sensor_msgs::image_encodings::MONO8;  // Convert color to gray
        }
        cv_ptr = cv_bridge::toCvCopy(image_msg, target_encoding);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    image_u8_t im = {
        .width = cv_ptr->image.cols,
        .height = cv_ptr->image.rows,
        .stride = cv_ptr->image.cols,
        .buf = cv_ptr->image.data
    };

    zarray_t* detections = apriltag_detector_detect(td_, &im);
    int num_dets = zarray_size(detections);
    if (num_dets > 0) {
        RCLCPP_INFO(this->get_logger(), "Found %d AprilTag(s)", num_dets);
    }

    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = image_msg->header;
    std::vector<int> ids;
    
    for (int i = 0; i < num_dets; ++i) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(this->get_logger(), "Tag detection: ID=%d, margin=%.2f, center=(%.1f,%.1f)", 
                     det->id, det->decision_margin, det->c[0], det->c[1]);

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tag_size_;
        info.fx = latest_camera_info_->k[0];
        info.fy = latest_camera_info_->k[4];
        info.cx = latest_camera_info_->k[2];
        info.cy = latest_camera_info_->k[5];

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        RCLCPP_DEBUG(this->get_logger(), "Pose error for tag %d: %.2f (threshold: %.1f)", 
                     det->id, err, pose_error_threshold_);

        if (err < pose_error_threshold_) {
            geometry_msgs::msg::Pose p;
            p.position.x = pose.t->data[0];
            p.position.y = pose.t->data[1];
            p.position.z = pose.t->data[2];

            tf2::Quaternion q;
            tf2::Matrix3x3 mat(pose.R->data[0], pose.R->data[1], pose.R->data[2],
                              pose.R->data[3], pose.R->data[4], pose.R->data[5],
                              pose.R->data[6], pose.R->data[7], pose.R->data[8]);
            mat.getRotation(q);
            p.orientation = tf2::toMsg(q);

            pose_array_msg.poses.push_back(p);
            ids.push_back(det->id);

            RCLCPP_INFO(this->get_logger(), "Valid pose for tag %d: pos=(%.3f, %.3f, %.3f)", 
                        det->id, p.position.x, p.position.y, p.position.z);
        } else {
            RCLCPP_WARN(this->get_logger(), "Pose error too high for tag %d: %.2f > %.1f", 
                        det->id, err, pose_error_threshold_);
        }

        // Clean up pose matrices
        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }

    if (!pose_array_msg.poses.empty()) {
        pose_array_pub_->publish(pose_array_msg);
        publishTransforms(pose_array_msg, image_msg->header, ids);
        publishMarkers(pose_array_msg, image_msg->header, ids, tag_size_);
        RCLCPP_INFO(this->get_logger(), "Published %zu valid tag poses", pose_array_msg.poses.size());
    }

    apriltag_detections_destroy(detections);
}

void AprilTagDetectorNode::publishTransforms(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids) {
    for (size_t i = 0; i < pose_array.poses.size(); ++i) {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header = header;
        transformStamped.child_frame_id = "tag_" + std::to_string(ids[i]);
        transformStamped.transform.translation.x = pose_array.poses[i].position.x;
        transformStamped.transform.translation.y = pose_array.poses[i].position.y;
        transformStamped.transform.translation.z = pose_array.poses[i].position.z;
        transformStamped.transform.rotation = pose_array.poses[i].orientation;
        tf_broadcaster_->sendTransform(transformStamped);
    }
}

void AprilTagDetectorNode::publishMarkers(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids, double tag_size) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < pose_array.poses.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "apriltag";
        marker.id = ids[i];
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose_array.poses[i];
        marker.scale.x = tag_size;
        marker.scale.y = tag_size;
        marker.scale.z = 0.1 * tag_size;
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // Persistent
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
