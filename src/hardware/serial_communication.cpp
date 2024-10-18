
#include "serial_communication.hpp"

#include <fcntl.h>
#include <unistd.h>

#include "shelfbot_utils.hpp"

namespace shelfbot {

SerialCommunication::SerialCommunication() : serial_fd_(-1) {
}

SerialCommunication::~SerialCommunication() {
  close();
}

bool SerialCommunication::open(const std::string& port) {
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  return serial_fd_ >= 0;
}

void SerialCommunication::close() {
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool SerialCommunication::writeCommandsToHardware(const std::vector<double>& hw_commands) {
  msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, hw_commands);

  ssize_t bytes_written = ::write(serial_fd_, sbuf.data(), sbuf.size());
  if (bytes_written != static_cast<ssize_t>(sbuf.size())) {
    log_error("SerialCommunication", "writeCommandsToHardware", "Failed to write to serial port");
    return false;
  }

  std::vector<uint8_t> response(64);
  ssize_t bytes_read = ::read(serial_fd_, response.data(), response.size());
  if (bytes_read <= 0) {
    log_error("SerialCommunication", "writeCommandsToHardware", "Failed to read from serial port");
    return false;
  }

  msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<char*>(response.data()), bytes_read);
  msgpack::object obj = oh.get();

  bool success;
  obj.convert(success);
  return success;
}

bool SerialCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
  msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, std::string("position"));

  ssize_t bytes_written = ::write(serial_fd_, sbuf.data(), sbuf.size());
  if (bytes_written != static_cast<ssize_t>(sbuf.size())) {
    log_error("SerialCommunication", "readStateFromHardware", "Failed to write to serial port");
    return false;
  }

  std::vector<uint8_t> response(64);
  ssize_t bytes_read = ::read(serial_fd_, response.data(), response.size());
  if (bytes_read <= 0) {
    log_error("SerialCommunication", "readStateFromHardware", "Failed to read from serial port");
    return false;
  }

  msgpack::object_handle oh = msgpack::unpack(reinterpret_cast<char*>(response.data()), bytes_read);
  msgpack::object obj = oh.get();

  obj.convert(hw_positions);
  return true;
}

}
