#ifndef SHELFBOT_UTILS_HPP
#define SHELFBOT_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace shelfbot {

class ShelfbotUtils {
 public:
  static void log_info(const std::string& logger_name, const std::string& tag, const std::string& message);
  static void log_error(const std::string& logger_name, const std::string& tag, const std::string& message);
  static void log_debug(const std::string& logger_name, const std::string& tag, const std::string& message);
};

} 

#endif