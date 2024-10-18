
#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <msgpack.hpp>
#include <string>
#include <vector>

namespace shelfbot {

class SerialCommunication {
 public:
  SerialCommunication();
  ~SerialCommunication();

  bool open(const std::string& port);
  void close();
  bool writeCommandsToHardware(const std::vector<double>& hw_commands);
  bool readStateFromHardware(std::vector<double>& hw_positions);

 private:
  int serial_fd_;
};

}

#endif
