#ifndef SHELFBOT_COMMS_H
#define SHELFBOT_COMMS_H

#include <string>
typedef std::string String;

// Define markers as 16-bit values
#define CMD_START_MARKER 0x3C3C  // "<<"
#define CMD_END_MARKER 0x3E3E    // ">>"
#define CMD_VALUE_START 0x7B7B   // "{{"
#define CMD_VALUE_END 0x7D7D     // "}}"
#define CMD_CHECKSUM_START 0x2323 // "##"
#define CMD_CHECKSUM_END 0x2424   // "$$"

enum CommandResponse : uint16_t {
    // Success responses
    RESP_OK = 0x4F4B,       // "OK"
    RESP_READY = 0x5244,    // "RD"
    RESP_COMPLETE = 0x434D,  // "CM"
    
    // Status responses  
    RESP_BUSY = 0x4255,     // "BU"
    RESP_MOVING = 0x4D56,   // "MV"
    RESP_STOPPED = 0x5354,  // "ST"
    
    // Error responses
    RESP_ERROR = 0x4552,    // "ER"
    RESP_INVALID = 0x494E,  // "IN"
    RESP_TIMEOUT = 0x544F,  // "TO"
    RESP_ERR_RANGE = 0x4552, // "RG" 
    RESP_ERR_VALUE = 0x4556, // "EV"
    RESP_ERR_PIN = 0x4550,   // "EP"
    RESP_ERR_MOTOR = 0x454D, // "EM"
    RESP_ERR_PARAM = 0x4548, // "EH"
    RESP_ERR_STATE = 0x4553, // "ES"
    RESP_ERR_COMM = 0x4543,  // "EC"
    RESP_ERR_CRC = 0x4352    // "CR"
};

enum CommandType : uint16_t {
    CMD_UNKNOWN = 0x3F3F,     // "??"
    CMD_GET_TEMP = 0x4754,    // "GT"
    CMD_SET_LED = 0x534C,     // "SL"
    CMD_READ_ADC = 0x5241,    // "RA"
    CMD_SET_PWM = 0x5350,     // "SP"
    CMD_GET_STATUS = 0x4753,  // "GS"
    CMD_SET_MOTOR_1 = 0x4D31, // "M1"
    CMD_SET_MOTOR_2 = 0x4D32, // "M2"
    CMD_SET_MOTOR_3 = 0x4D33, // "M3"
    CMD_SET_MOTOR_4 = 0x4D34, // "M4"
    CMD_SET_MOTOR_5 = 0x4D35, // "M5"
    CMD_SET_MOTOR_6 = 0x4D36, // "M6"
    CMD_GET_MOTOR_1_POS = 0x5031, // "P1"
    CMD_GET_MOTOR_2_POS = 0x5032, // "P2"
    CMD_GET_MOTOR_3_POS = 0x5033, // "P3"
    CMD_GET_MOTOR_4_POS = 0x5034, // "P4"
    CMD_GET_MOTOR_5_POS = 0x5035, // "P5"
    CMD_GET_MOTOR_6_POS = 0x5036, // "P6"
    CMD_GET_MOTOR_1_VEL = 0x5631, // "V1"
    CMD_GET_MOTOR_2_VEL = 0x5632, // "V2"
    CMD_GET_MOTOR_3_VEL = 0x5633, // "V3"
    CMD_GET_MOTOR_4_VEL = 0x5634, // "V4"
    CMD_GET_MOTOR_5_VEL = 0x5635, // "V5"
    CMD_GET_MOTOR_6_VEL = 0x5636, // "V6"
    CMD_STOP_MOTOR_1 = 0x5331, // "S1"
    CMD_STOP_MOTOR_2 = 0x5332, // "S2"
    CMD_STOP_MOTOR_3 = 0x5333, // "S3"
    CMD_STOP_MOTOR_4 = 0x5334, // "S4"
    CMD_STOP_MOTOR_5 = 0x5335, // "S5"
    CMD_STOP_MOTOR_6 = 0x5336, // "S6"
    CMD_STOP_ALL = 0x5341,     // "SA"
    CMD_GET_BATTERY = 0x4742,  // "GB"
    CMD_GET_SYSTEM = 0x5359    // "SY"
};

class ShelfbotComms {
public:
    static void begin();
    static void handleComms();
    static String handleCommand(char* message);
    static void sendCommand(CommandType cmd, uint16_t value = 0);
    static String formatResponse(CommandResponse resp, const String& value);
    static String formatCommand(CommandType cmd, const String& value);
    static String parseValue(const String& message);
    static CommandType parseCommand(const String& message);
    static bool verifyChecksum(const String& message);
    static void moveAllMotors(long position);

private:
    static String getTemperature();
    static String setLed(const String& value);
    static String readAdc(const String& value);
    static String setPwm(const String& value);
    static String getStatus();
    static String setMotor(uint8_t index, const String& value);
    static String getMotorPosition(uint8_t index);
    static String getMotorVelocity(uint8_t index);
    static String stopMotor(uint8_t index);
    static String stopAllMotors();
    static String getBatteryLevel();
    static String getSystemStatus();
};

#endif