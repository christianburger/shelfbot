#ifndef REST_COMMUNICATION_HPP
#define REST_COMMUNICATION_HPP

#include "communication_interface.hpp"
#include "shelfbot_comms.h"
#include <curl/curl.h>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace shelfbot {

class RestCommunication : public CommunicationInterface {

public:
    RestCommunication();
    ~RestCommunication();

    bool open(const std::string& base_url);
    void close();
    bool writeCommandsToHardware(const std::vector<double>& hw_commands);
    bool readStateFromHardware(std::vector<double>& hw_positions);
    
    std::string setMotorPosition(uint8_t index, long position);
    void moveAllMotors(long position, long speed, bool nonBlocking);

private:
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp);
    std::string formatResponse(CommandResponse resp, const std::string& value);
    std::string makeRestCall(const std::string& endpoint, const std::string& method, const nlohmann::json& data);

    bool is_open_;
    std::string base_url_;
    std::vector<double> hw_positions_;
    CURL* curl_;
};

}

#endif