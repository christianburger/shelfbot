#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher") {

    this->declare_parameter<std::string>("camera_info_url", "");
    this->declare_parameter<std::string>("camera_name",     "esp32_cam");
    // frame_id must match the optical frame in the URDF so that
    // image_pipeline, apriltag_ros and ORB-SLAM3 all agree on the
    // coordinate frame of the incoming images.
    this->declare_parameter<std::string>("frame_id",        "camera_link_optical_frame");
    this->declare_parameter<int>   ("image_width",    800);
    this->declare_parameter<int>   ("image_height",   600);
    this->declare_parameter<double>("focal_length",   800.0);

    std::string camera_info_url;
    this->get_parameter("camera_info_url", camera_info_url);
    this->get_parameter("camera_name",     camera_name_);
    this->get_parameter("frame_id",        frame_id_);
    this->get_parameter("image_width",     image_width_);
    this->get_parameter("image_height",    image_height_);
    this->get_parameter("focal_length",    focal_length_);

    info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_info_url);

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/image_raw",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    // Subscribe to compressed images from micro-ROS / ESP32-CAM
    compressed_image_sub_ =
        this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/esp32_cam/image_raw/compressed",
            rclcpp::SensorDataQoS(),
            std::bind(&CameraPublisher::image_callback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Camera Publisher initialised:");
    RCLCPP_INFO(this->get_logger(), "  Image size : %dx%d",
                image_width_, image_height_);
    RCLCPP_INFO(this->get_logger(), "  Frame ID   : %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  Subscribing: /esp32_cam/image_raw/compressed");
    RCLCPP_INFO(this->get_logger(),
                "  Publishing : camera/image_raw, camera/camera_info");
  }

private:
  // ── Subscriber callback ────────────────────────────────────────────
  void image_callback(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(compressed_msg,
                                   sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "cv_bridge decompress exception: %s", e.what());
      return;
    }

    // Decompressed image
    sensor_msgs::msg::Image image_msg;
    cv_ptr->toImageMsg(image_msg);
    image_msg.header          = compressed_msg->header;
    image_msg.header.frame_id = frame_id_;

    // Camera info (from file or defaults)
    sensor_msgs::msg::CameraInfo camera_info_msg =
        info_manager_->isCalibrated()
            ? info_manager_->getCameraInfo()
            : createDefaultCameraInfo();

    // Synchronise timestamp and frame_id with the image
    camera_info_msg.header.stamp    = image_msg.header.stamp;
    camera_info_msg.header.frame_id = frame_id_;
    camera_info_msg.width           = static_cast<uint32_t>(image_width_);
    camera_info_msg.height          = static_cast<uint32_t>(image_height_);

    // Intrinsic matrix K
    // [fx  0  cx]
    // [ 0 fy  cy]
    // [ 0  0   1]
    const double cx = image_width_  / 2.0;
    const double cy = image_height_ / 2.0;
    camera_info_msg.k = {
      focal_length_, 0.0,           cx,
      0.0,           focal_length_, cy,
      0.0,           0.0,           1.0
    };

    // Projection matrix P (no stereo baseline)
    // [fx  0  cx  0]
    // [ 0 fy  cy  0]
    // [ 0  0   1  0]
    camera_info_msg.p = {
      focal_length_, 0.0,           cx, 0.0,
      0.0,           focal_length_, cy, 0.0,
      0.0,           0.0,           1.0, 0.0
    };

    image_publisher_->publish(image_msg);
    info_publisher_->publish(camera_info_msg);

    static int count = 0;
    if (++count % 25 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Published image+camera_info pair #%d  stamp: %d.%09d",
                  count,
                  camera_info_msg.header.stamp.sec,
                  camera_info_msg.header.stamp.nanosec);
    }
  }

  // ── Default camera info (used when no calibration file is loaded) ──
  sensor_msgs::msg::CameraInfo createDefaultCameraInfo() const {
    sensor_msgs::msg::CameraInfo ci;
    ci.height             = static_cast<uint32_t>(image_height_);
    ci.width              = static_cast<uint32_t>(image_width_);
    ci.distortion_model   = "plumb_bob";
    ci.d                  = {0.0, 0.0, 0.0, 0.0, 0.0};

    const double cx = image_width_  / 2.0;
    const double cy = image_height_ / 2.0;

    std::fill(ci.k.begin(), ci.k.end(), 0.0);
    ci.k[0] = focal_length_;  // fx
    ci.k[2] = cx;
    ci.k[4] = focal_length_;  // fy
    ci.k[5] = cy;
    ci.k[8] = 1.0;

    // Rectification: identity (monocular)
    std::fill(ci.r.begin(), ci.r.end(), 0.0);
    ci.r[0] = ci.r[4] = ci.r[8] = 1.0;

    std::fill(ci.p.begin(), ci.p.end(), 0.0);
    ci.p[0]  = focal_length_;  // fx
    ci.p[2]  = cx;
    ci.p[5]  = focal_length_;  // fy
    ci.p[6]  = cy;
    ci.p[10] = 1.0;

    return ci;
  }

  // ── Members ────────────────────────────────────────────────────────
  std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  info_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_image_sub_;

  std::string camera_name_;
  std::string frame_id_;
  int         image_width_;
  int         image_height_;
  double      focal_length_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
