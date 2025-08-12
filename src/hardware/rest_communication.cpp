#include "rest_communication.hpp"
#include "shelfbot_utils.hpp"
#include "shelfbot_comms.h"
#include <nlohmann/json.hpp>
#include <sstream>

namespace shelfbot {

size_t RestCommunication::WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    log_debug("RestCommunication", "WriteCallback", "Received " + std::to_string(size * nmemb) + " bytes");
    return size * nmemb;
}

RestCommunication::RestCommunication() : is_open_(false), hw_positions_(4, 0.0) {
    curl_ = curl_easy_init();
    log_info("RestCommunication", "Constructor", "Initializing REST communication");
    if (!curl_) {
        log_error("RestCommunication", "Constructor", "Failed to initialize CURL");
    }
}

RestCommunication::~RestCommunication() {
    log_info("RestCommunication", "Destructor", "Cleaning up REST communication");
    close();
    if (curl_) {
        curl_easy_cleanup(curl_);
        log_debug("RestCommunication", "Destructor", "CURL cleanup completed");
    }
}

bool RestCommunication::open(const std::string& base_url) {
    base_url_ = base_url;
    is_open_ = true;
    log_info("RestCommunication", "open", "Opening REST connection to: " + base_url);
    return true;
}

void RestCommunication::close() {
    if (is_open_) {
        log_info("RestCommunication", "close", "Closing REST connection to: " + base_url_);
        is_open_ = false;
    }
}

std::string RestCommunication::makeRestCall(const std::string& endpoint, const std::string& method, const nlohmann::json& data) {
    if (!is_open_ || !curl_) {
        log_error("RestCommunication", "makeRestCall",
            "Connection state: " + std::string(is_open_ ? "open" : "closed") +
            ", CURL state: " + std::string(curl_ ? "initialized" : "null"));
        return "";
    }

    // Handle trailing/leading slashes to avoid double slashes
    std::string clean_base = base_url_;
    if (!clean_base.empty() && clean_base.back() == '/') {
        clean_base.pop_back();
    }
    std::string url = clean_base + endpoint;
    std::string response;
    log_info("RestCommunication", "makeRestCall", "REQUEST: " + method + " " + url);

    curl_easy_reset(curl_);
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "Accept: application/json");
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);

    std::string json_str;
    if (!data.empty()) {
        json_str = data.dump();
        log_info("RestCommunication", "makeRestCall", "REQUEST PAYLOAD:\n" + json_str);
    }

    switch(method[0]) {
        case 'P':
            if (method.compare("POST") == 0) {
                curl_easy_setopt(curl_, CURLOPT_POST, 1L);
                if (!json_str.empty()) {
                    log_info("RestCommunication", "makeRestCall", "PAYLOAD LENGTH: " + std::to_string(json_str.length()));
                    log_info("RestCommunication", "makeRestCall", "PAYLOAD RAW: " + json_str);
                    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_str.c_str());
                    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, json_str.length());
                }
            } else if (method.compare("PUT") == 0 || method.compare("PATCH") == 0) {
                curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, method.c_str());
                if (!json_str.empty()) {
                    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_str.c_str());
                }
            }
            break;

        case 'G':
            if (method.compare("GET") == 0) {
                curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
            }
            break;

        case 'D':
            if (method.compare("DELETE") == 0) {
                curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, method.c_str());
            }
            break;

        default:
            log_error("RestCommunication", "makeRestCall", "Unsupported HTTP method: " + method);
            curl_slist_free_all(headers);
            return "";
    }

    long http_code = 0;
    CURLcode res = curl_easy_perform(curl_);
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
    curl_slist_free_all(headers);

    if (res != CURLE_OK) {
        log_error("RestCommunication", "makeRestCall",
            "CURL ERROR: " + std::string(curl_easy_strerror(res)) +
            "\nFailed URL: " + url);
        return "";
    }

    log_info("RestCommunication", "makeRestCall",
        "RESPONSE CODE: " + std::to_string(http_code) +
        "\nRESPONSE BODY:\n" + response);

    return response;
}

std::string RestCommunication::setMotorPosition(uint8_t index, long position) {
    log_info("RestCommunication", "setMotorPosition", "Setting motor " + std::to_string(index) + " to position " + std::to_string(position));

    nlohmann::json json_data = {
        {"motor", index},
        {"position", position},
        {"speed", 4000}
    };

    std::string response = makeRestCall("/motor", "POST", json_data);
    
    if (response.empty()) {
        log_error("RestCommunication", "setMotorPosition", "Failed to set motor " + std::to_string(index));
        return formatResponse(RESP_ERR_MOTOR, std::to_string(index));
    }
    
    return formatResponse(RESP_MOVING, response);
}

void RestCommunication::moveAllMotors(long position, long speed, bool nonBlocking) {
    log_info("RestCommunication", "moveAllMotors", "Moving all motors to position " + std::to_string(position) + " at speed " + std::to_string(speed) + " (nonBlocking: " + std::string(nonBlocking ? "true" : "false") + ")");

    nlohmann::json json_data = {
        {"positions", std::vector<long>(6, position)},  // Array of 6 positions
        {"speed", speed},
        {"nonBlocking", nonBlocking}
    };
    makeRestCall("/motors", "POST", json_data);
}

bool RestCommunication::writeCommandsToHardware(const std::vector<double>& hw_commands) {
    log_info("RestCommunication", "writeCommandsToHardware", "Writing commands to hardware");
    
    if (hw_commands.empty() || hw_commands.size() > 6) {
        log_error("RestCommunication", "writeCommandsToHardware", "Invalid number of commands: " + std::to_string(hw_commands.size()));
        return false;
    }

    if (!is_open_ || !curl_) {
        log_error("RestCommunication", "writeCommandsToHardware", "REST connection not open");
        return false;
    }

    // Create JSON with only the positions array
    nlohmann::json json_data = {
        {"positions", std::vector<double>()}
    };
    
    // Fill positions array
    auto& positions = json_data["positions"];
    for (const auto& cmd : hw_commands) {
        positions.push_back(cmd);  // Keep as double for precision
    }
    
    // Pad remaining positions with zeros if needed
    while (positions.size() < 6) {
        positions.push_back(0);
    }

    // Make REST call with simplified JSON
    std::string response = makeRestCall("/motors", "POST", json_data);
    
    if (response.empty()) {
        return false;
    }

    hw_positions_ = hw_commands;
    return true;
}

bool RestCommunication::readStateFromHardware(std::vector<double>& hw_positions) {
    log_info("RestCommunication", "readStateFromHardware", "Reading hardware state");
    
    std::string response = makeRestCall("/status", "GET", nlohmann::json());
    
    if (response.empty()) {
        log_error("RestCommunication", "readStateFromHardware", "Failed to read state - empty response");
        return false;
    }

    try {
        auto json_response = nlohmann::json::parse(response);
        
        std::stringstream ss;
        ss << "Positions: [";
        for (size_t i = 0; i < hw_positions_.size(); ++i) {
            hw_positions_[i] = json_response["motors"][i]["position"];
            ss << hw_positions_[i];
            if (i < hw_positions_.size() - 1) ss << ", ";
        }
        ss << "]";
        log_debug("RestCommunication", "readStateFromHardware", ss.str());
        
        hw_positions = hw_positions_;
        return true;
    } catch (const std::exception& e) {
        log_error("RestCommunication", "readStateFromHardware", 
                 "Failed to parse response: " + std::string(e.what()) + 
                 "\nResponse was: " + response);
        return false;
    }
}

std::string RestCommunication::formatResponse(CommandResponse resp, const std::string& value) {
    std::string response;
    response.reserve(32);
    response += (char)(resp >> 8);
    response += (char)(resp & 0xFF);
    response += value;
    
    log_debug("RestCommunication", "formatResponse", 
              "Response code: 0x" + std::to_string(resp) + 
              ", Value: " + value);
    
    return response;
}

}