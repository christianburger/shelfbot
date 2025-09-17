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
        this->declare_parameter<std::string>("camera_name", "esp32_cam");
        this->declare_parameter<std::string>("frame_id", "camera_link");
        
        // ESP32-CAM specific parameters
        this->declare_parameter<int>("image_width", 800);
        this->declare_parameter<int>("image_height", 600);
        this->declare_parameter<double>("focal_length", 800.0);
        
        std::string camera_info_url;
        this->get_parameter("camera_info_url", camera_info_url);
        this->get_parameter("camera_name", camera_name_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("image_width", image_width_);
        this->get_parameter("image_height", image_height_);
        this->get_parameter("focal_length", focal_length_);

        info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Subscribe only to compressed images from Micro-ROS
        compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/esp32_cam/image_raw/compressed", rclcpp::SensorDataQoS(), std::bind(&CameraPublisher::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Camera Publisher initialized:");
        RCLCPP_INFO(this->get_logger(), "  Image size: %dx%d", image_width_, image_height_);
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Subscribed to: /esp32_cam/image_raw/compressed");
        RCLCPP_INFO(this->get_logger(), "  Publishing image + generated camera_info to: /camera/image_raw and /camera/camera_info");
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg) {
        try {
            // Decompress compressed image to cv::Mat (BGR8)
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(compressed_msg, sensor_msgs::image_encodings::BGR8);

            // Create decompressed image message
            sensor_msgs::msg::Image image_msg;
            cv_ptr->toImageMsg(image_msg);
            image_msg.header = compressed_msg->header;  // Preserve original header
            image_msg.header.frame_id = frame_id_;

            // Get CameraInfo (from file or default)
            sensor_msgs::msg::CameraInfo camera_info_msg;
            if (info_manager_->isCalibrated()) {
                camera_info_msg = info_manager_->getCameraInfo();
                RCLCPP_DEBUG(this->get_logger(), "Using calibration from file");
            } else {
                camera_info_msg = createDefaultCameraInfo();
                RCLCPP_DEBUG(this->get_logger(), "Using default ESP32-CAM calibration");
            }

            // Synchronize timestamp and frame_id with image
            camera_info_msg.header.stamp = image_msg.header.stamp;
            camera_info_msg.header.frame_id = frame_id_;

            // Override intrinsics with calibrated values
            camera_info_msg.width = image_width_;
            camera_info_msg.height = image_height_;
            camera_info_msg.k = {focal_length_, 0.0, image_width_ / 2.0, 0.0, focal_length_, image_height_ / 2.0, 0.0, 0.0, 1.0};
            camera_info_msg.p = {focal_length_, 0.0, image_width_ / 2.0, 0.0, 0.0, focal_length_, image_height_ / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};

            // Publish image + generated camera_info
            image_publisher_->publish(image_msg);
            info_publisher_->publish(camera_info_msg);

            // Log occasionally
            static int count = 0;
            if (++count % 25 == 0) {  // Every 5 seconds at 5Hz
                RCLCPP_INFO(this->get_logger(), "Published image + camera_info pair #%d - timestamp: %d.%09d", 
                           count, camera_info_msg.header.stamp.sec, camera_info_msg.header.stamp.nanosec);
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception during decompression: %s", e.what());
        }
    }
    
    sensor_msgs::msg::CameraInfo createDefaultCameraInfo() {
        sensor_msgs::msg::CameraInfo camera_info;
        
        camera_info.height = image_height_;
        camera_info.width = image_width_;
        camera_info.distortion_model = "plumb_bob";
        
        // Default distortion (no distortion)
        camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Camera matrix K = [fx  0 cx]
        //                   [ 0 fy cy]
        //                   [ 0  0  1]
        std::fill(camera_info.k.begin(), camera_info.k.end(), 0.0);
        camera_info.k[0] = focal_length_;           // fx
        camera_info.k[2] = image_width_ / 2.0;      // cx (image center)
        camera_info.k[4] = focal_length_;           // fy  
        camera_info.k[5] = image_height_ / 2.0;     // cy (image center)
        camera_info.k[8] = 1.0;
        
        // Rectification matrix R (identity for single camera)
        std::fill(camera_info.r.begin(), camera_info.r.end(), 0.0);
        camera_info.r[0] = camera_info.r[4] = camera_info.r[8] = 1.0;
        
        // Projection matrix P
        std::fill(camera_info.p.begin(), camera_info.p.end(), 0.0);
        camera_info.p[0] = focal_length_;           // fx
        camera_info.p[2] = image_width_ / 2.0;      // cx
        camera_info.p[5] = focal_length_;           // fy
        camera_info.p[6] = image_height_ / 2.0;     // cy
        camera_info.p[10] = 1.0;
        
        return camera_info;
    }

    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    std::string camera_name_;
    std::string frame_id_;
    int image_width_;
    int image_height_;
    double focal_length_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
