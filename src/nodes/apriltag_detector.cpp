#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/header.hpp"
#include <memory>
#include <sstream>
#include <iomanip>

#include "shelfbot/shelfbot_utils.hpp"
#include "shelfbot/apriltag_manager.hpp"

#include "apriltag.h"
#include "tag36h11.h"
#include "apriltag_pose.h"
#include "common/matd.h"

namespace shelfbot {

class AprilTagDetectorNode : public rclcpp::Node {

public:
    AprilTagDetectorNode();
    ~AprilTagDetectorNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    bool validatePoseInputs(double tagsize, double fx, double fy, double cx, double cy, int tag_id);
    bool validatePose(const apriltag_pose_t& pose, double error, int tag_id);

    apriltag_family_t*   tf_;
    apriltag_detector_t* td_;
    std::unique_ptr<AprilTagManager> tag_manager_;

    // Both subscribers use RELIABLE to match camera_publisher's output QoS.
    // image_transport subscriber uses the "raw" transport on /camera/image_raw.
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
    bool camera_info_received_ = false;

    double tag_size_;
    double pose_error_threshold_;
};

AprilTagDetectorNode::AprilTagDetectorNode() : Node("apriltag_detector_node") {
    this->declare_parameter<double>("tag_size",             0.16);
    this->declare_parameter<double>("pose_error_threshold", 100.0);
    tag_size_             = this->get_parameter("tag_size").as_double();
    pose_error_threshold_ = this->get_parameter("pose_error_threshold").as_double();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "tag_size=" << tag_size_
        << ", pose_error_threshold=" << pose_error_threshold_;
    shelfbot::log_info(this->get_name(), "Constructor", oss.str());

    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);
    tag_manager_ = std::make_unique<AprilTagManager>(this);

    // ── QoS ────────────────────────────────────────────────────────────────
    // camera_publisher publishes /camera/image_raw and /camera/camera_info
    // with RELIABLE, VOLATILE, depth=10.  We must use the same reliability
    // so the DDS match succeeds.  BEST_EFFORT (SensorDataQoS) would be
    // silently dropped by a RELIABLE publisher in ROS 2.
    // image_transport wraps the rmw profile; we pass it explicitly.
    // ───────────────────────────────────────────────────────────────────────
    rmw_qos_profile_t reliable_profile = rmw_qos_profile_default;
    reliable_profile.history           = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    reliable_profile.depth             = 10;
    reliable_profile.reliability       = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    reliable_profile.durability        = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    // image_transport subscriber — absolute topic /camera/image_raw
    image_sub_ = image_transport::create_subscription(
        this,
        "/camera/image_raw",
        std::bind(&AprilTagDetectorNode::imageCallback, this,
                  std::placeholders::_1),
        "raw",
        reliable_profile);

    // camera_info subscriber — absolute topic /camera/camera_info
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                            .reliable()
                            .durability_volatile();

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info",
        reliable_qos,
        std::bind(&AprilTagDetectorNode::cameraInfoCallback, this,
                  std::placeholders::_1));

    shelfbot::log_info(this->get_name(), "Constructor",
        "Subscribing /camera/image_raw, /camera/camera_info  (RELIABLE, VOLATILE)");
    shelfbot::log_info(this->get_name(), "Constructor",
        "AprilTagManager initialised — publishing /tag_poses (RELIABLE), "
        "/apriltag_markers (RELIABLE)");
}

AprilTagDetectorNode::~AprilTagDetectorNode() {
    shelfbot::log_info(this->get_name(), "Destructor", "Shutting down");
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
}

void AprilTagDetectorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
    latest_camera_info_ = info_msg;
    if (!camera_info_received_) {
        camera_info_received_ = true;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "width=" << info_msg->width
            << ", height=" << info_msg->height
            << ", fx=" << info_msg->k[0]
            << ", fy=" << info_msg->k[4]
            << ", cx=" << info_msg->k[2]
            << ", cy=" << info_msg->k[5];
        shelfbot::log_info(this->get_name(), "CameraInfoCallback", oss.str());
    }
}

bool AprilTagDetectorNode::validatePoseInputs(
    double tagsize, double fx, double fy, double cx, double cy, int tag_id)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "Tag " << tag_id
        << " — tagsize=" << tagsize
        << ", fx=" << fx << ", fy=" << fy
        << ", cx=" << cx << ", cy=" << cy;
    shelfbot::log_info(this->get_name(), "PoseEstimation", oss.str());

    if (tagsize <= 0.0) {
        shelfbot::log_warn(this->get_name(), "PoseEstimation",
            "Invalid tag size; skipping");
        return false;
    }
    if (fx <= 0.0 || fy <= 0.0) {
        shelfbot::log_warn(this->get_name(), "PoseEstimation",
            "Invalid camera intrinsics (fx/fy <= 0); skipping");
        return false;
    }
    if (cx <= 0.0 || cy <= 0.0) {
        shelfbot::log_warn(this->get_name(), "PoseEstimation",
            "Principal point (cx/cy) zero or negative; results may be inaccurate");
    }
    return true;
}

bool AprilTagDetectorNode::validatePose(
    const apriltag_pose_t& pose, double error, int tag_id)
{
    std::ostringstream oss_err;
    oss_err << std::fixed << std::setprecision(2)
            << "Tag " << tag_id
            << " — error=" << error
            << " (threshold=" << pose_error_threshold_ << ")";
    shelfbot::log_info(this->get_name(), "PoseEstimation", oss_err.str());

    if (pose.R && pose.t) {
        std::ostringstream oss_t;
        oss_t << std::fixed << std::setprecision(4)
              << "Tag " << tag_id
              << " — t=[" << pose.t->data[0] << ", "
              << pose.t->data[1] << ", "
              << pose.t->data[2] << "]";
        shelfbot::log_info(this->get_name(), "PoseEstimation", oss_t.str());
    } else {
        shelfbot::log_warn(this->get_name(), "PoseEstimation",
            "Tag " + std::to_string(tag_id) + " — pose matrices invalid");
        return false;
    }

    if (error < pose_error_threshold_) {
        return true;
    }
    shelfbot::log_warn(this->get_name(), "PoseEstimation",
        "Tag " + std::to_string(tag_id) + " — pose rejected: error "
        + std::to_string(error) + " > threshold "
        + std::to_string(pose_error_threshold_));
    return false;
}

void AprilTagDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
{
    if (!camera_info_received_ || !latest_camera_info_) {
        shelfbot::log_warn(this->get_name(), "ImageCallback",
            "No camera info yet — skipping frame");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg,
                                     sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        shelfbot::log_error(this->get_name(), "ImageCallback",
            std::string("cv_bridge: ") + e.what());
        return;
    }

    image_u8_t im = {
        .width  = cv_ptr->image.cols,
        .height = cv_ptr->image.rows,
        .stride = cv_ptr->image.cols,
        .buf    = cv_ptr->image.data
    };

    zarray_t* detections = apriltag_detector_detect(td_, &im);

    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = image_msg->header;   // preserves SNTP epoch stamp
    std::vector<int> ids;

    for (int i = 0; i < zarray_size(detections); ++i) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        if (!det->H) continue;

        apriltag_detection_info_t info;
        info.det     = det;
        info.tagsize = tag_size_;
        info.fx      = latest_camera_info_->k[0];
        info.fy      = latest_camera_info_->k[4];
        info.cx      = latest_camera_info_->k[2];
        info.cy      = latest_camera_info_->k[5];

        if (!validatePoseInputs(info.tagsize, info.fx, info.fy,
                                info.cx, info.cy, det->id)) {
            continue;
        }

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        if (validatePose(pose, err, det->id)) {
            geometry_msgs::msg::Pose p;
            p.position.x = pose.t->data[0];
            p.position.y = pose.t->data[1];
            p.position.z = pose.t->data[2];

            tf2::Quaternion q;
            tf2::Matrix3x3 mat(
                pose.R->data[0], pose.R->data[1], pose.R->data[2],
                pose.R->data[3], pose.R->data[4], pose.R->data[5],
                pose.R->data[6], pose.R->data[7], pose.R->data[8]);
            mat.getRotation(q);
            p.orientation = tf2::toMsg(q);

            pose_array_msg.poses.push_back(p);
            ids.push_back(det->id);
        }

        if (pose.R) matd_destroy(pose.R);
        if (pose.t) matd_destroy(pose.t);
    }

    if (!pose_array_msg.poses.empty()) {
        tag_manager_->updateTags(pose_array_msg, image_msg->header,
                                 ids, tag_size_);
    }

    apriltag_detections_destroy(detections);
}

} // namespace shelfbot

int main(int argc, char* argv[]) {
    using shelfbot::AprilTagDetectorNode;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
