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

bool MockCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    //log_trace("MockCommunication", "readStateFromHardware", "Enter method");
    
    if (!is_open_) {
        log_error("MockCommunication", "readStateFromHardware", "Mock port is not open");
        return false;
    }
    
    //log_trace("MockCommunication", "readStateFromHardware", "Current positions before copy: [" + 
    //          std::to_string(hw_positions_[0]) + "," + 
    //          std::to_string(hw_positions_[1]) + "," + 
    //          std::to_string(hw_positions_[2]) + "," + 
    //          std::to_string(hw_positions_[3]) + "]");
              
    hw_positions = hw_positions_;
    
    //log_trace("MockCommunication", "readStateFromHardware", "Positions copied successfully");
    //log_trace("MockCommunication", "readStateFromHardware", "Exit method");
    
    return true;
}

bool MockCommunication::writeCommandsToHardware(const std::vector<double>& hw_commands) {
    //log_trace("MockCommunication", "writeCommandsToHardware", "Enter method");
    
    if (!is_open_) {
        log_error("MockCommunication", "writeCommandsToHardware", "Mock port is not open");
        return false;
    }

    //log_trace("MockCommunication", "writeCommandsToHardware", "Current commands: [" + 
    //          std::to_string(hw_commands[0]) + "," + 
    //         std::to_string(hw_commands[1]) + "," + 
    //          std::to_string(hw_commands[2]) + "," + 
    //          std::to_string(hw_commands[3]) + "]");

    hw_positions_ = hw_commands;
    
    //log_trace("MockCommunication", "writeCommandsToHardware", "Commands stored successfully");
    //log_trace("MockCommunication", "writeCommandsToHardware", "Exit method");
    
    return true;
}
}
