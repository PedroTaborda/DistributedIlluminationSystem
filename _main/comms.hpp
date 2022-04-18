#ifndef COMMS_HPP
#define COMMS_HPP

#include "parser.hpp"
#include <Arduino.h>

#define SDA_MASTER_PIN 16
#define SCL_MASTER_PIN 17
#define SDA_SLAVE_PIN 14
#define SCL_SLAVE_PIN 15



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

enum MSG_TYPE : unsigned char
{
    MSG_TYPE_ERROR=0,
    MSG_TYPE_NONE,
    MSG_TYPE_COMMAND,
    MSG_TYPE_REPLY,
    MSG_TYPE_STREAM
};

enum ProcessingResult
{
    PROCESSING_OK=0,
    PROCESSING_I2C_TIMEOUT,
    PROCESSING_I2C_OTHER_ERROR,
    PROCESSING_LOCAL_EXECUTION_FAILURE,
};

const int receivedDataBufferSize = 64; // to be put in a central place
const unsigned long timeout_ms = 50;
const signed char addr_offset = 8;

class Comms
{
public:
    CommandParser parser;
    signed char my_id = 0;

    Comms(){};
    Comms(CommandParser parser) : parser(parser) {}

    void init();

    bool joinNetwork();

    ProcessingResult processCommand(const char *command);

    void eventLoop();

private:
    char receivedData[receivedDataBufferSize];
    char receivedDataSize = 0;
    MSG_TYPE receivedMsgType = MSG_TYPE_NONE;

    bool error = false;
    char errorMsg[100];

    void onReceive(int);
    void onRequest();

    void flushError();
    void processReceivedData();
};

#endif // COMMS_HPP
