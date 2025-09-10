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
#include <sstream>
#include <iomanip>

#include "shelfbot/shelfbot_utils.hpp"

// The core apriltag C library headers
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "apriltag_pose.h"
#include "common/matd.h" // For logging homography matrix
}


namespace shelfbot {

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

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "Parameters - tag_size: " << tag_size_ << ", pose_error_threshold: " << pose_error_threshold_;
    shelfbot::log_info(this->get_name(), "Constructor", oss.str());

    // Initialize Apriltag detector
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    shelfbot::log_info(this->get_name(), "Constructor", "AprilTag detector initialized with tag36h11 family");

    // Publishers
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tag_poses", rclcpp::SensorDataQoS());
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tag_markers", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    shelfbot::log_info(this->get_name(), "Constructor", "Publishers created: /tag_poses, /tag_markers");

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

    shelfbot::log_info(this->get_name(), "Constructor", "Separate subscribers created for image_raw and camera_info");
}

AprilTagDetectorNode::~AprilTagDetectorNode() {
    shelfbot::log_info(this->get_name(), "Destructor", "AprilTag Detector Shutting Down");
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
}

void AprilTagDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    latest_camera_info_ = info_msg;
    if (!camera_info_received_) {
        camera_info_received_ = true;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << "Camera info received - width: " << info_msg->width << ", height: " << info_msg->height << ", fx: " << info_msg->k[0] << ", fy: " << info_msg->k[4] << ", cx: " << info_msg->k[2] << ", cy: " << info_msg->k[5];
        shelfbot::log_info(this->get_name(), "CameraInfoCallback", oss.str());
    }
}

void AprilTagDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    // Check if we have camera info
    if (!camera_info_received_ || !latest_camera_info_) {
        shelfbot::log_warn(this->get_name(), "ImageCallback", "No camera info available yet - skipping image");
        return;
    }

    std::ostringstream oss_image;
    oss_image << "Processing image - size: " << image_msg->width << "x" << image_msg->height << ", encoding: " << image_msg->encoding << ", timestamp: " << image_msg->header.stamp.sec << "." << std::setfill('0') << std::setw(9) << image_msg->header.stamp.nanosec;
    shelfbot::log_info(this->get_name(), "ImageCallback", oss_image.str());

    cv_bridge::CvImagePtr cv_ptr;
    try {
        // Handle common encodings: prefer MONO8 for tags, but convert from bgr8/rgb8 if needed
        std::string target_encoding = sensor_msgs::image_encodings::MONO8;
        if (image_msg->encoding == sensor_msgs::image_encodings::BGR8 || 
            image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
            target_encoding = sensor_msgs::image_encodings::MONO8;  // Convert color to gray
        }
        cv_ptr = cv_bridge::toCvCopy(image_msg, target_encoding);
        std::ostringstream oss_convert;
        oss_convert << "Image converted to " << target_encoding << ", size: " << cv_ptr->image.cols << "x" << cv_ptr->image.rows;
        shelfbot::log_info(this->get_name(), "ImageCallback", oss_convert.str());
    } catch (cv_bridge::Exception& e) {
        std::ostringstream oss_err;
        oss_err << "cv_bridge exception: " << e.what();
        shelfbot::log_error(this->get_name(), "ImageCallback", oss_err.str());
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
    std::ostringstream oss_dets;
    oss_dets << "Detected " << num_dets << " AprilTag(s)";
    shelfbot::log_info(this->get_name(), "ImageCallback", oss_dets.str());

    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = image_msg->header;
    std::vector<int> ids;
    
    for (int i = 0; i < num_dets; ++i) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        
        // Log detection details
        std::ostringstream oss_det;
        oss_det << std::fixed << std::setprecision(2) << "Tag " << i << " - ID: " << det->id << ", margin: " << det->decision_margin << ", center: (" << det->c[0] << ", " << det->c[1] << ")";
        shelfbot::log_info(this->get_name(), "Detection", oss_det.str());
        
        // Log homography matrix
        if (det->H) {
            std::ostringstream oss_h;
            oss_h << std::fixed << std::setprecision(4) << "Tag " << det->id << " - Homography: [" << det->H->data[0] << ", " << det->H->data[1] << ", " << det->H->data[2] << "; " << det->H->data[3] << ", " << det->H->data[4] << ", " << det->H->data[5] << "; " << det->H->data[6] << ", " << det->H->data[7] << ", " << det->H->data[8] << "]";
            shelfbot::log_info(this->get_name(), "Detection", oss_h.str());
        } else {
            std::ostringstream oss_null;
            oss_null << "Tag " << det->id << " - Homography matrix is NULL";
            shelfbot::log_warn(this->get_name(), "Detection", oss_null.str());
            continue; // Skip if homography is invalid
        }

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tag_size_;
        info.fx = latest_camera_info_->k[0];
        info.fy = latest_camera_info_->k[4];
        info.cx = latest_camera_info_->k[2];
        info.cy = latest_camera_info_->k[5];

        // Log pose estimation inputs
        std::ostringstream oss_input;
        oss_input << std::fixed << std::setprecision(3) << "Tag " << det->id << " - Pose input: tagsize: " << info.tagsize << ", fx: " << info.fx << ", fy: " << info.fy << ", cx: " << info.cx << ", cy: " << info.cy;
        shelfbot::log_info(this->get_name(), "PoseEstimation", oss_input.str());

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        // Log pose estimation output
        std::ostringstream oss_err;
        oss_err << std::fixed << std::setprecision(2) << "Tag " << det->id << " - Pose error: " << err << " (threshold: " << pose_error_threshold_ << ")";
        shelfbot::log_info(this->get_name(), "PoseEstimation", oss_err.str());
        
        if (pose.R && pose.t) {
            std::ostringstream oss_r;
            oss_r << std::fixed << std::setprecision(4) << "Tag " << det->id << " - Rotation: [" << pose.R->data[0] << ", " << pose.R->data[1] << ", " << pose.R->data[2] << "; " << pose.R->data[3] << ", " << pose.R->data[4] << ", " << pose.R->data[5] << "; " << pose.R->data[6] << ", " << pose.R->data[7] << ", " << pose.R->data[8] << "]";
            shelfbot::log_info(this->get_name(), "PoseEstimation", oss_r.str());
            std::ostringstream oss_t;
            oss_t << std::fixed << std::setprecision(4) << "Tag " << det->id << " - Translation: [" << pose.t->data[0] << ", " << pose.t->data[1] << ", " << pose.t->data[2] << "]";
            shelfbot::log_info(this->get_name(), "PoseEstimation", oss_t.str());
        } else {
            std::ostringstream oss_invalid;
            oss_invalid << "Tag " << det->id << " - Pose matrices invalid (R: " << (void*)pose.R << ", t: " << (void*)pose.t << ")";
            shelfbot::log_warn(this->get_name(), "PoseEstimation", oss_invalid.str());
        }

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

            std::ostringstream oss_valid;
            oss_valid << std::fixed << std::setprecision(3) << "Tag " << det->id << " - Valid pose: pos=(" << p.position.x << ", " << p.position.y << ", " << p.position.z << "), quat=(" << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << ", " << p.orientation.w << ")";
            shelfbot::log_info(this->get_name(), "PoseEstimation", oss_valid.str());
        } else {
            std::ostringstream oss_reject;
            oss_reject << std::fixed << std::setprecision(2) << "Tag " << det->id << " - Pose rejected: error " << err << " > threshold " << pose_error_threshold_;
            shelfbot::log_warn(this->get_name(), "PoseEstimation", oss_reject.str());
        }

        // Clean up pose matrices
        if (pose.R) matd_destroy(pose.R);
        if (pose.t) matd_destroy(pose.t);
    }

    if (!pose_array_msg.poses.empty()) {
        pose_array_pub_->publish(pose_array_msg);
        publishTransforms(pose_array_msg, image_msg->header, ids);
        publishMarkers(pose_array_msg, image_msg->header, ids, tag_size_);
        std::ostringstream oss_pub;
        oss_pub << "Published " << pose_array_msg.poses.size() << " valid tag poses";
        shelfbot::log_info(this->get_name(), "Publishing", oss_pub.str());
    } else {
        shelfbot::log_info(this->get_name(), "Publishing", "No valid poses to publish");
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
        std::ostringstream oss_tf;
        oss_tf << std::fixed << std::setprecision(3) << "Published transform for tag_" << ids[i] << ": pos=(" << transformStamped.transform.translation.x << ", " << transformStamped.transform.translation.y << ", " << transformStamped.transform.translation.z << ")";
        shelfbot::log_info(this->get_name(), "PublishTransforms", oss_tf.str());
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
        std::ostringstream oss_marker;
        oss_marker << std::fixed << std::setprecision(3) << "Published marker for tag_" << ids[i] << ": scale=(" << marker.scale.x << ", " << marker.scale.y << ", " << marker.scale.z << ")";
        shelfbot::log_info(this->get_name(), "PublishMarkers", oss_marker.str());
    }
    marker_pub_->publish(marker_array);
}
} //shelfbot namespace

int main(int argc, char* argv[]) {
  using shelfbot::AprilTagDetectorNode;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
