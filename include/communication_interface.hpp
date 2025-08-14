#pragma once

#include <string>
#include <vector>

namespace shelfbot {

class CommunicationInterface {
public:
    virtual ~CommunicationInterface() = default;
    
    virtual bool open(const std::string& connection_string) = 0;
    virtual void close() = 0;
    virtual bool writeCommandsToHardware(const std::vector<double>& hw_commands) = 0;
    virtual bool writeSpeedsToHardware(const std::vector<double>& hw_speeds) = 0;
    virtual bool readStateFromHardware(std::vector<double>& hw_positions) = 0;
};

}
