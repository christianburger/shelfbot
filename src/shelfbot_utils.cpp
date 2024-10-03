#include "shelfbot_utils.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace shelfbot {

void log_info(const std::string& logger_name, const std::string& tag, const std::string& message) {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
  RCLCPP_INFO(rclcpp::get_logger(logger_name), "[%s] %s: %s", ss.str().c_str(), tag.c_str(), message.c_str());
}

void log_error(const std::string& logger_name, const std::string& tag, const std::string& message) {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
  RCLCPP_ERROR(rclcpp::get_logger(logger_name), "[%s] %s: %s", ss.str().c_str(), tag.c_str(), message.c_str());
}

void log_debug(const std::string& logger_name, const std::string& tag, const std::string& message) {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
  RCLCPP_DEBUG(rclcpp::get_logger(logger_name), "[%s] %s: %s", ss.str().c_str(), tag.c_str(), message.c_str());
}

} 