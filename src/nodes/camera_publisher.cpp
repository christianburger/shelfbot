#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

class CameraInfoPublisher : public rclcpp::Node {
public:
    CameraInfoPublisher() : Node("camera_info_publisher") {
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
        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::SensorDataQoS());

        // Match image publishing rate (ESP32-CAM typically 5-10 FPS)
        // 5Hz to match image rate
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CameraInfoPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Camera Info Publisher initialized:");
        RCLCPP_INFO(this->get_logger(), "  Image size: %dx%d", image_width_, image_height_);
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing rate: 5Hz");
    }

private:
    void timer_callback() {
        sensor_msgs::msg::CameraInfo camera_info_msg;
        
        // Always publish, even without calibration file
        if (info_manager_->isCalibrated()) {
            // Use calibration from file if available
            camera_info_msg = info_manager_->getCameraInfo();
            RCLCPP_DEBUG(this->get_logger(), "Using calibration from file");
        } else {
            // Create default camera info for ESP32-CAM
            camera_info_msg = createDefaultCameraInfo();
            RCLCPP_DEBUG(this->get_logger(), "Using default ESP32-CAM calibration");
        }
        
        // Always update timestamp and frame_id
        camera_info_msg.header.stamp = this->now();
        camera_info_msg.header.frame_id = frame_id_;
        
        publisher_->publish(camera_info_msg);
        
        // Log occasionally to verify publishing
        static int count = 0;
        if (++count % 25 == 1) { // Every 5 seconds at 5Hz
            RCLCPP_INFO(this->get_logger(), 
                       "Published CameraInfo #%d - timestamp: %d.%09d", 
                       count, camera_info_msg.header.stamp.sec, 
                       camera_info_msg.header.stamp.nanosec);
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
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string camera_name_;
    std::string frame_id_;
    int image_width_;
    int image_height_;
    double focal_length_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}
