#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher() : Node("camera_info_publisher")
    {
        this->declare_parameter<std::string>("camera_info_url", "");
        this->declare_parameter<std::string>("camera_name", "camera");
        this->declare_parameter<std::string>("frame_id", "camera_link");

        std::string camera_info_url;
        this->get_parameter("camera_info_url", camera_info_url);
        this->get_parameter("camera_name", camera_name_);
        this->get_parameter("frame_id", frame_id_);

        info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url);
        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CameraInfoPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (info_manager_->isCalibrated())
        {
            sensor_msgs::msg::CameraInfo camera_info_msg = info_manager_->getCameraInfo();
            camera_info_msg.header.stamp = this->now();
            camera_info_msg.header.frame_id = frame_id_;
            RCLCPP_INFO(this->get_logger(), "Publishing CameraInfo with timestamp: %d.%d", 
                        camera_info_msg.header.stamp.sec, camera_info_msg.header.stamp.nanosec);
            publisher_->publish(camera_info_msg);
        }
    }

    std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string camera_name_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}
