// hardware_interface.cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hardware_interface_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
