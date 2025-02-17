#include "mock_communication.hpp"
#include <msgpack.hpp>

namespace shelfbot {

MockCommunication::MockCommunication() : is_open_(false), hw_positions_(4, 0.0) {
  log_info("MockCommunication", "Constructor", "Initializing mock communication");
}

MockCommunication::~MockCommunication() {
  log_info("MockCommunication", "Destructor", "Closing mock communication");
  close();
}

bool MockCommunication::open(const std::string& port) {
  log_info("MockCommunication", "open", "Opening mock port: " + port);
  is_open_ = true;
  return true;
}

void MockCommunication::close() {
  if (is_open_) {
    log_info("MockCommunication", "close", "Closing mock port");
    is_open_ = false;
  }
}

bool MockCommunication::writeCommandsToHardware(const std::vector<double>& hw_commands) {
  if (!is_open_) {
    log_error("MockCommunication", "writeCommandsToHardware", "Mock port is not open");
    return false;
  }

  log_info("MockCommunication", "writeCommandsToHardware", 
            "Writing commands: [" + std::to_string(hw_commands[0]) + "," + 
            std::to_string(hw_commands[1]) + "," + 
            std::to_string(hw_commands[2]) + "," + 
            std::to_string(hw_commands[3]) + "]");

  hw_positions_ = hw_commands;
  return true;
}

bool MockCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
  if (!is_open_) {
    log_error("MockCommunication", "readStateFromHardware", "Mock port is not open");
    return false;
  }

  log_info("MockCommunication", "readStateFromHardware", 
           "Current positions: [" + std::to_string(hw_positions_[0]) + "," + 
           std::to_string(hw_positions_[1]) + "," + 
           std::to_string(hw_positions_[2]) + "," + 
           std::to_string(hw_positions_[3]) + "]");
    
  log_info("MockCommunication", "readStateFromHardware", "Reading state from mock hardware"); 

  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] += 0.01 * std::sin(rclcpp::Clock().now().seconds());  // Simulate small oscillations
  }
  hw_positions = hw_positions_;
  return true;
}
}
