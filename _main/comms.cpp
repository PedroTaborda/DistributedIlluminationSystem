#include "calibration.hpp"
#include "comms.hpp"
#include "parser.hpp"
#include <Wire.h>

Comms *_comms;

extern int myID;

void Comms::init()
{
    Wire.setSDA(SDA_MASTER_PIN);
    Wire.setSCL(SCL_MASTER_PIN);
    Wire.begin();
    Wire1.setSDA(SDA_SLAVE_PIN);
    Wire1.setSCL(SCL_SLAVE_PIN);

    Wire.setTimeout(timeout_ms);
    Wire1.setTimeout(timeout_ms);
    _comms = this;
}

bool Comms::joinNetwork()
{
    int ret;
    for (int my_potential_addr = my_id + addr_offset; my_potential_addr < 128; my_potential_addr++)
    {
        do
        {
            Wire.beginTransmission(my_potential_addr);
            ret = Wire.endTransmission(false);
        } while (ret == 4);

        if (ret == 0)
        {
            my_id = my_potential_addr - addr_offset;
            myID = my_id;
            Wire1.begin(my_potential_addr);
            Wire1.onReceive([](int i){ _comms->onReceive(i); });
            Wire1.onRequest([](){ _comms->onRequest(); });
            do
            {
                Wire.beginTransmission(0x0);
                Wire.write(MSG_TYPE_WAKEUP);
                ret = Wire.endTransmission(true);
            } while(ret == 4);

            if(my_id == 0)
                calibrator.resetWait();
            return true;
        }
    }
    return false;
}

void Comms::calibrateNetwork() {
    // Warn everybody calibration is starting
    do {
        Wire.beginTransmission(0x0);
        Wire.write(MSG_TYPE_BEGIN_CALIBRATION);
    } while(Wire.endTransmission(true) == 4);
    // Become the maestro
    calibrator.becomeMaestro();

    // Everybody should know that now only calibration messages
    // are being traded. Every luminaire in the network now
    // replies with their id.
    // Block while waiting for the ids to stop arriving,
    // hijacking the eventLoop.
    calibrator.resetWaitId();
    while(calibrator.waitingIds()) eventLoop();
    Serial.print("Calibration starting.\n");
    // After the highest id has been found, order everyone to calibrate,
    // in order.
    for(signed char i = 0; i <= calibrator.getHighestId(); i++) {
        // Order luminaire i to run its calibration cycle. Also
        // lets the other luminaires know that i is about to run
        // its calibration cycle in order to calibrate coupled gains.
        Serial.printf("Calibrating luminaire %d\n", i);
        do {
            Wire.beginTransmission(0x0);
            Wire.write(MSG_TYPE_CALIBRATE_ID);
            Wire.write(i);
        } while(Wire.endTransmission(true) == 4);

        // When the message gets sent, its either our time to calibrate
        // or we should look.
        if(i == my_id)
            calibrator.selfCalibrate(my_id);
        else
            calibrator.calibrateGainId(i);

        // Wait for the node that just ran its calibration cycle to turn off
        // (if it wasn't me) and then give some slack for messaging delays.
        if(i != my_id)
            delay(STEADY_STATE_WAIT_MS);
        delay(MESSAGE_SLACK_WAIT_MS);
    }

    // Let everyone know calibration is over.
    do {
        Wire.beginTransmission(0x0);
        Wire.write(MSG_TYPE_END_CALIBRATION);
    } while(Wire.endTransmission(true) == 4);

    calibrator.endCalibration();
}

ProcessingResult Comms::processCommand(const char *command)
{
    signed char luminaireID = parser.getLuminaireId(command);
    int ret = 0;
    char *commandRet = NULL;

    //Serial.printf("My id: %d, luminaire id: %d\n", my_id, luminaireID);
    if (luminaireID == my_id)
    {
        commandRet = parser.executeCommand(command);
        Serial.println(commandRet == NULL ? "NULL" : commandRet);
        return commandRet == NULL ? PROCESSING_LOCAL_EXECUTION_FAILURE : PROCESSING_OK;
    }

    const char *strippedCommand = parser.strip(command);
    // Serial.printf("Sending command: '%s'\n", strippedCommand);
    Wire.beginTransmission(luminaireID + addr_offset);
    Wire.write((uint8_t) MSG_TYPE_COMMAND);
    while (*strippedCommand)
    {
        Wire.write(*strippedCommand);
        strippedCommand++;
    }
    ret = Wire.endTransmission();
    if (ret == 0){
        return PROCESSING_OK;
    }
    else if (ret == 5){
        return PROCESSING_I2C_TIMEOUT;
    }
    else{
        return PROCESSING_I2C_OTHER_ERROR;
    }
}

void Comms::onReceive(int signed bytesReceived)
{
    MSG_TYPE msgType = (MSG_TYPE) Wire1.read();
    //error = true;
    //sprintf(errorMsg, "Received %d bytes. Message type: %d\n", bytesReceived, msgType);

    if (bytesReceived == 1)
        return;

    if (bytesReceived > receivedDataBufferSize)
    {
        error = true;
        sprintf(errorMsg, "Received %d/%d bytes. Message type: %d. Buffer too small.\n", bytesReceived, receivedDataBufferSize, msgType);
        return;
    }
    for (int buf_idx = 0; buf_idx < bytesReceived - 1; buf_idx++)
    {
        receivedData[buf_idx] = Wire1.read();
        // Serial.print(receivedData[buf_idx]);
    }
    // Serial.print("\n");
    receivedData[bytesReceived] = '\0';

    receivedMsgType = msgType;
    receivedDataSize = bytesReceived - 1;
}

void Comms::onRequest()
{
    Wire.write(1);
}

void Comms::flushError(){
    if (error)
        Serial.printf("%s\n", errorMsg);
    error = false;
}

void Comms::processReceivedData()
{
    if (receivedMsgType == MSG_TYPE_NONE)
        return;

    char *commandRet = NULL;
    switch (receivedMsgType)
    {
    // If a command was issued to me, I will execute it and reply with the result.
    case MSG_TYPE_COMMAND:
        commandRet = parser.executeCommand(receivedData);
        Wire.beginTransmission(0);
        Wire.write(MSG_TYPE_REPLY);
        Wire.write(commandRet);
        Wire.endTransmission();
        break;

    // If a reply is coming my way, I will relay it to the Serial interface.
    // Same for stream.
    case MSG_TYPE_REPLY:
    case MSG_TYPE_STREAM:
        Serial.printf("%s\n", receivedData);
        break;
        
    // In case someone has just woken up and I'm id=0, I'll see if we're waiting
    // to start calibration. If yes, reset the counter back to the start.
    case MSG_TYPE_WAKEUP:
        if(my_id == 0x0 && calibrator.waiting()) {
            calibrator.resetWait();
        }
        break;

    case MSG_TYPE_BEGIN_CALIBRATION:
        // The maestro ignores its own calls to calibrate
        if(!calibrator.isMaestro()) {
            // Broadcast our id so the highest id can be determined
            do {
                Wire.beginTransmission(0x0);
                Wire.write(MSG_TYPE_FIND_HIGHEST_ID);
                Wire.write(my_id);
            } while(Wire.endTransmission(true) == 4);
        } else {
            calibrator.setHighestId(my_id);
            calibrator.resetWaitId();
        }
        break;

    case MSG_TYPE_FIND_HIGHEST_ID:
        Serial.printf("Received highest id: %d\n", receivedData[0]);
        calibrator.setHighestId((signed char) receivedData[0]);
        calibrator.resetWaitId();
        break;

    case MSG_TYPE_CALIBRATE_ID:
        // The maestro ignores its own calls to calibrate
        if(!calibrator.isMaestro()) {
            if(receivedData[0] != my_id)
                calibrator.calibrateGainId(receivedData[0]);
            else
                calibrator.selfCalibrate(my_id);
        }
        break;

    case MSG_TYPE_END_CALIBRATION:
        // The maestro ignores its own calls to calibrate
        if(!calibrator.isMaestro()) {
            calibrator.endCalibration();
        }

    default:
        break;
    }
    receivedMsgType = MSG_TYPE_NONE;
}

void Comms::eventLoop()
{
    flushError();
    processReceivedData();
}
