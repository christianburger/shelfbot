#ifndef SHELFBOT_UTILS_HPP
#define SHELFBOT_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace shelfbot {

void log_info(const std::string& logger_name, const std::string& tag, const std::string& message);
void log_warn(const std::string& logger_name, const std::string& tag, const std::string& message);
void log_error(const std::string& logger_name, const std::string& tag, const std::string& message);
void log_debug(const std::string& logger_name, const std::string& tag, const std::string& message);
void log_trace(const std::string& logger_name, const std::string& tag, const std::string& message);
}

#endif