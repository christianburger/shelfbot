#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// The core apriltag C library headers
extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "apriltag_pose.h"
}

class AprilTagDetectorNode : public rclcpp::Node
{
public:
    AprilTagDetectorNode();
    ~AprilTagDetectorNode();

private:
    // Member Functions
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

    void publishTransforms(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids);

    void publishMarkers(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids, double tag_size);

    // Member Variables
    apriltag_family_t* tf_;
    apriltag_detector_t* td_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    image_transport::CameraSubscriber camera_sub_;
};

AprilTagDetectorNode::AprilTagDetectorNode() : Node("apriltag_detector_node")
{
    // Initialize Apriltag detector
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    // Publishers and Broadcaster
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tag_poses", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tag_markers", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscriber
    camera_sub_ = image_transport::create_camera_subscription(
        this, "image_rect",
        std::bind(&AprilTagDetectorNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2),
        "raw");
}

AprilTagDetectorNode::~AprilTagDetectorNode()
{
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
}

void AprilTagDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
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
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = image_msg->header;
    std::vector<int> ids;
    
    double tag_size = 0.16;

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tag_size;
        info.fx = info_msg->k[0];
        info.fy = info_msg->k[4];
        info.cx = info_msg->k[2];
        info.cy = info_msg->k[5];

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        if (err < 100)
        {
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
        }
    }

    if (!pose_array_msg.poses.empty())
    {
        pose_array_pub_->publish(pose_array_msg);
        publishTransforms(pose_array_msg, image_msg->header, ids);
        publishMarkers(pose_array_msg, image_msg->header, ids, tag_size);
    }

    apriltag_detections_destroy(detections);
}

void AprilTagDetectorNode::publishTransforms(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids)
{
    for (size_t i = 0; i < pose_array.poses.size(); ++i)
    {
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

void AprilTagDetectorNode::publishMarkers(const geometry_msgs::msg::PoseArray& pose_array, const std_msgs::msg::Header& header, const std::vector<int>& ids, double tag_size)
{
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < pose_array.poses.size(); ++i)
    {
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
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}