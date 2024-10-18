
#ifndef MOCK_COMMUNICATION_HPP
#define MOCK_COMMUNICATION_HPP

#include <vector>
#include <string>
#include "shelfbot_utils.hpp"

namespace shelfbot {

class MockCommunication {
public:
    MockCommunication();
    ~MockCommunication();

    bool open(const std::string& port);
    void close();
    bool writeCommandsToHardware(const std::vector<double>& hw_commands);
    bool readStateFromHardware(std::vector<double>& hw_positions);

private:
    bool is_open_;
    std::vector<double> hw_positions_;
};

}

#endif
