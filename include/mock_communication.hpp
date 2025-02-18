#ifndef MOCK_COMMUNICATION_HPP
#define MOCK_COMMUNICATION_HPP

#include "communication_interface.hpp"
#include "shelfbot_utils.hpp"
#include <string>
#include <vector>

namespace shelfbot {

class MockCommunication : public CommunicationInterface {
public:
    MockCommunication();
    ~MockCommunication() override;

    bool open(const std::string& port) override;
    void close() override;
    bool writeCommandsToHardware(const std::vector<double>& hw_commands) override;
    bool readStateFromHardware(std::vector<double>& hw_positions) override;

private:
    bool is_open_;
    std::vector<double> hw_positions_;
};

}

#endif