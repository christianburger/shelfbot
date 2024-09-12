#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraDriver : public rclcpp::Node
{
public:
  CameraDriver() : Node("camera_driver")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CameraDriver::publish_image, this));
  }

private:
  void publish_image()
  {
    auto message = sensor_msgs::msg::Image();
    // Fill the message with dummy data for now
    message.header.stamp = this->now();
    message.header.frame_id = "camera_frame";
    message.height = 480;
    message.width = 640;
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = message.width * 3;
    message.data.resize(message.step * message.height);

    publisher_->publish(message);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriver>());
  rclcpp::shutdown();
  return 0;
}
