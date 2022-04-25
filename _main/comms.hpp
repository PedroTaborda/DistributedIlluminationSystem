#ifndef COMMS_HPP
#define COMMS_HPP

#include "parser.hpp"
#include <Arduino.h>

#define SEND_MSG(addr, timeout_ms, WRITE_STATEMENTS, ret)       \
    {                                                           \
        long t0 = millis();                                     \
        do                                                      \
        {                                                       \
            Wire.beginTransmission(addr);                       \
            WRITE_STATEMENTS                                    \
            ret = Wire.endTransmission(timeout_ms);             \
        } while ((ret == 4) && ((millis() - t0) < timeout_ms)); \
    }

#ifdef ZE
const int SDA_MASTER_PIN = 0;
const int SCL_MASTER_PIN = 1;
const int SDA_SLAVE_PIN = 2;
const int SCL_SLAVE_PIN = 3;
#else
const int SDA_MASTER_PIN = 16;
const int SCL_MASTER_PIN = 17;
const int SDA_SLAVE_PIN = 14;
const int SCL_SLAVE_PIN = 15;
#endif



/* This file contains the specification for communication between the
 * luminaires.
 * There are four main conversation types:
 * 1. Wake up. All luminaires are identified and addresses are assigned.
 * 2. Callibration. Cross gain computation is performed.
 * 3. Control. The information related to the consensus algorithm is sent.
 * 4. Commands. The messages exchanged come from a Serial interface (if present)
 *   and are parsed and executed.
 *      If the target luminaire is the current luminaire, there is no I2C involved.
 *      If the target luminaire is not the current luminaire, the message is
 *   stripped of unnecessary spaces and sent through I2C to the address
 *   <id> + offset (8 by default). The response is relayed back to the Serial
 *   interface.
 *      Some commands start a stream of data. This data is broadcasted to all
 *   luminaires, and the response is relayed back to the Serial interface, if
 *   present.
 */

enum MSG_TYPE : unsigned char {
    MSG_TYPE_ERROR = 0,
    MSG_TYPE_NONE,
    MSG_TYPE_COMMAND,
    MSG_TYPE_REPLY,
    MSG_TYPE_REPLY_RAW,
    MSG_TYPE_STREAM,
    MSG_TYPE_ANNOUNCE_ID,
    MSG_TYPE_BEGIN_CALIBRATION,
    MSG_TYPE_ROLL_CALL,
    MSG_TYPE_CALIBRATE_ID,
    MSG_TYPE_END_CALIBRATION,
    MSG_TYPE_BUFFER,
    MSG_TYPE_BUFFER_END,
    MSG_TYPE_VERIFY_LIST,
    MSG_TYPE_VERIFY_LIST_NACK,
    MSG_TYPE_CONSENSUS_START,
    MSG_TYPE_CONSENSUS_D,
    MSG_TYPE_CONSENSUS_CONVERGENCE,
};

enum ProcessingResult
{
    PROCESSING_OK=0,
    PROCESSING_I2C_TIMEOUT,
    PROCESSING_I2C_OTHER_ERROR,
    PROCESSING_LOCAL_EXECUTION_FAILURE,
};

inline constexpr int receivedDataBufferSize = 64; // to be put in a central place
inline constexpr unsigned long TIMEOUT_MS = 20;
inline constexpr unsigned int RETRY_MULTIPLIER = 10;
inline constexpr unsigned long RETRY_TIMEOUT_MS = RETRY_MULTIPLIER * TIMEOUT_MS;
inline constexpr unsigned long VERIFY_WAIT_MS = 100;
inline constexpr unsigned long ROLL_CALL_WAIT_MS = 100;
inline constexpr signed char addr_offset = 8;
inline constexpr unsigned long MESSAGE_SLACK_WAIT_MS = 500;

bool receivingBuffer = false;

class Comms
{
public:
    CommandParser parser;
    signed char my_id = 0;

    Comms(){};
    Comms(CommandParser parser) : parser(parser) {}

    void init() volatile;

    bool joinNetwork() volatile;
    void calibrateNetwork() volatile;

    ProcessingResult processCommand(const char *command) volatile;

    void eventLoop() volatile;

private:
    uint8_t receivedData[receivedDataBufferSize];
    uint8_t receivedDataSize = 0;
    MSG_TYPE receivedMsgType = MSG_TYPE_NONE;

    bool error = false;
    char errorMsg[100];

    void onReceive(int) volatile;
    void onRequest() volatile;

    void flushError() volatile;
    void processReceivedData() volatile; 

    bool successfulRegister = true;
    bool waitVerify = true;
    bool waitRollCall = true;
    alarm_id_t verifyAlarm = -1;
    alarm_id_t rollCallAlarm = -1;

    void startVerifyAckAlarm() volatile;
    void startRollCallAlarm() volatile;
};

void parseSerial(volatile Comms& comms);

#endif // COMMS_HPP
