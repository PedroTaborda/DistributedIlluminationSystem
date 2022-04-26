#include "buffer.hpp"
#include "calibration.hpp"
#include "comms.hpp"
#include "consensus.hpp"
#include "globals.hpp"
#include "network.hpp"
#include "parser.hpp"
#include <Wire.h>

volatile Comms* _comms;
int asked = 1;

bool receivingBuffer = false;

bool streamLuminance = false;
bool streamDuty = false;
bool streamSampleTime = false;
bool streamIntegralError = false;
bool streamTrackingError = false;
bool streamSimulator = false;
bool streamReference = false;

void Comms::init() volatile
{
    Wire.setSDA(SDA_MASTER_PIN);
    Wire.setSCL(SCL_MASTER_PIN);
    Wire.begin();
    Wire1.setSDA(SDA_SLAVE_PIN);
    Wire1.setSCL(SCL_SLAVE_PIN);

    Wire.setTimeout(TIMEOUT_MS);
    Wire1.setTimeout(TIMEOUT_MS);
    _comms = this;
}

bool Comms::joinNetwork() volatile
{
    int ret;
    for (int my_potential_addr = my_id + addr_offset; my_potential_addr < 128; my_potential_addr++)
    {
        SEND_MSG(my_potential_addr, RETRY_TIMEOUT_MS, ,ret)
        DEBUG_PRINT("Address %d done: ret = %d\n", my_potential_addr, ret)

        // If no one was at the address being probed, it is now
        // ours.
        if (ret == 4)
        {
            my_id = my_potential_addr - addr_offset;
            myID = my_id;
            Wire1.onReceive([](int i){ _comms->onReceive(i); });
            Wire1.onRequest([](){ _comms->onRequest(); });
            Wire1.begin(my_potential_addr);
            
            SEND_MSG(0, RETRY_TIMEOUT_MS,
                Wire.write(MSG_TYPE_ANNOUNCE_ID);
                Wire.write(myID);,
            ret)
            DEBUG_PRINT("Broadcasting wakeup as id %hhu\n", (uint8_t)my_id)

            network.addNodeToNetwork((uint8_t)myID);

            do {
                successfulRegister = true;
                SEND_MSG(0, RETRY_TIMEOUT_MS,
                    Wire.write(MSG_TYPE_VERIFY_LIST);
                    Wire.write(network.getNumberNodesNetwork());
                    for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++)
                        Wire.write(network.getNetwork()[i]);,
                ret)

                startVerifyAckAlarm();

                while(waitVerify) eventLoop();
            
                if(!successfulRegister) {
                    SEND_MSG(0, RETRY_TIMEOUT_MS,
                        Wire.write(MSG_TYPE_ROLL_CALL);,
                    ret)

                    startRollCallAlarm();

                    while(waitRollCall) eventLoop();
                }
            } while(successfulRegister == false);

            for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++) {
                DEBUG_PRINT("%hhuth member of the network is %hhu\n", i, network.getNetwork()[i])
            }

            if(my_id == 0)
                calibrator.resetWait();

            return true;
        }
        else
        {
            network.addNodeToNetwork(my_potential_addr - addr_offset);
        }
    }
    return false;
}

void Comms::calibrateNetwork() volatile{
    // Warn everybody calibration is starting
    int ret;
    SEND_MSG(0, RETRY_TIMEOUT_MS,
        Wire.write(MSG_TYPE_BEGIN_CALIBRATION);,
    ret)
    // Become the maestro
    calibrator.becomeMaestro();
    DEBUG_PRINT("Calibration starting.\n");
    // After the highest id has been found, order everyone to calibrate,
    // in order.
    for(signed char i = 0; i < network.getNumberNodesNetwork(); i++) {
        // Order luminaire i to run its calibration cycle. Also
        // lets the other luminaires know that i is about to run
        // its calibration cycle in order to calibrate coupled gains.
        DEBUG_PRINT("Calibrating luminaire %d\n", network.getNetwork()[i]);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_CALIBRATE_ID);
            Wire.write(network.getNetwork()[i]);,
        ret)
        // When the message gets sent, its either our time to calibrate
        // or we should look.
        if(network.getNetwork()[i] == my_id)
            calibrator.selfCalibrate(my_id);
        else
            calibrator.calibrateGainId(i);

        // Wait for the node that just ran its calibration cycle to turn off
        // (if it wasn't me) and then give some slack for messaging delays.
        if(network.getNetwork()[i] != my_id)
            delay(STEADY_STATE_WAIT_MS);
        delay(MESSAGE_SLACK_WAIT_MS);
    }

    // Let everyone know calibration is over.
    SEND_MSG(0, RETRY_TIMEOUT_MS,
        Wire.write(MSG_TYPE_END_CALIBRATION);,
    ret)

    DEBUG_PRINT("Calibrated %d luminaires.\n", network.getNumberNodesNetwork())
    DEBUG_PRINT("Calibration complete.\n")
}

ProcessingResult Comms::processCommand(const char *command) volatile
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

    const char *strippedCommand = command;//= parser.strip(command);
    // Serial.printf("Sending command: '%s'\n", strippedCommand);
    Wire.beginTransmission(luminaireID + addr_offset);
    DEBUG_PRINT("Sending MSG_TYPE_COMMAND")
    Wire.write(myID);
    Wire.write(MSG_TYPE_COMMAND);
    while (*strippedCommand)
    {
        Wire.write(*strippedCommand);
        strippedCommand++;
    }
    Wire.write('\0');
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

void Comms::onReceive(int signed bytesReceived) volatile
{
    signed char id = (MSG_TYPE) Wire1.read();
    MSG_TYPE msgType = (MSG_TYPE) Wire1.read();
    messageCounter += 1;
    //error = true;
    //sprintf(errorMsg, "Received %d bytes. Message type: %d\n", bytesReceived, msgType);

    //DEBUG_PRINT("Received message %hhu with %d bytes", msgType, bytesReceived)

    if (bytesReceived + 1 > receivedDataBufferSize)
    {
        error = true;
        sprintf((char *)errorMsg, "Received %d/%d bytes. Message type: %d. Buffer too small.\n", bytesReceived, receivedDataBufferSize, msgType);
        return;
    }

    receivedMsgTypeBuffer.insert(msgType);
    receivedDataSize = 0;
    receivedMsgDataBuffer[dataBufferHead][0] = id;
    receivedMsgDataBuffer[dataBufferHead][1] = '\0';
    if (bytesReceived == 2) {
        dataBufferHead = (MSG_BUFFER_SIZE + dataBufferHead + 1) % MSG_BUFFER_SIZE;
        dataBufferItems = min(MSG_BUFFER_SIZE, dataBufferItems + 1);
        return;
    }
    
    for (int buf_idx = 1; buf_idx < bytesReceived; buf_idx++)
    {
        receivedMsgDataBuffer[dataBufferHead][buf_idx] = (uint8_t)Wire1.read();
    }
    receivedMsgDataBuffer[dataBufferHead][bytesReceived] = '\0';
    dataBufferHead = (MSG_BUFFER_SIZE + dataBufferHead + 1) % MSG_BUFFER_SIZE;
    dataBufferItems = min(MSG_BUFFER_SIZE, dataBufferItems + 1);
    receivedDataSize = bytesReceived;
}

void Comms::onRequest() volatile
{
    Wire1.write(consensus.state == CONSENSUS_STATE_WAITING_FOR_NEIGHBORS);

    if(consensus.state == CONSENSUS_STATE_WAITING_CONSENSUS)
        Wire1.write((uint8_t *)consensus.di, sizeof(double) * network.getNumberNodesNetwork());

    asked += 1;
    if(asked == network.getNumberNodesNetwork()) {
        consensus.setState(CONSENSUS_STATE_WAITING_CONSENSUS);
        asked = 1;
    }
}

void Comms::flushError() volatile{
    if (error)
        Serial.printf("%s\n", errorMsg);
    error = false;
}

void Comms::streamVars() volatile{
    int sampleNum = controller.getSampleNumber();
    if (lastSampleStreamed == sampleNum)
        return;

    lastSampleStreamed = sampleNum;

    sample_t newSample = controller.getSample();
    int ret=0;

    if(streamDuty){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s d %d %.4f", myID, newSample.u);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }
    if(streamLuminance){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s l %d %.4f", myID, newSample.L);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }
    if(streamSampleTime){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s T %d %.4f", myID, newSample.time);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }
    if(streamIntegralError){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s i %d %.4f", myID, newSample.IntegralError);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }
    if(streamTrackingError){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s t %d %.4f", myID, newSample.TrackingError);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }
    if(streamSimulator){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s s %d %.4f", myID, newSample.SimulatorValue);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }
    if(streamReference){
        snprintf((char *)streamVarsBuffer, MSG_BUFFER_SIZE, "s r %d %.4f", myID, newSample.Reference);
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY);
            Wire.write((char *)streamVarsBuffer);,
        ret
        )
    }

}

void Comms::processReceivedData() volatile
{
    noInterrupts();
    MSG_TYPE receivedMsg; 
    if(receivedMsgTypeBuffer.available())
        receivedMsg = receivedMsgTypeBuffer.popEnd();
    else
        receivedMsg = MSG_TYPE_NONE;

    volatile uint8_t *receivedDataBuffer;
    if(dataBufferItems != 0)
        receivedDataBuffer = receivedMsgDataBuffer[(MSG_BUFFER_SIZE + dataBufferHead - dataBufferItems--) % MSG_BUFFER_SIZE];
    interrupts();

    if (receivedMsg == MSG_TYPE_NONE)
        return;

    processedMessageCounter += 1;
    DEBUG_PRINT("Received message = %li, Processeced received message = %li\n", messageCounter, processedMessageCounter)

    int ret;
    char *commandRet = NULL;
    switch (receivedMsg)
    {
    // If a command was issued to me, I will execute it and reply with the result.
    case MSG_TYPE_COMMAND:
        commandRet = parser.executeCommand((const char *)(receivedDataBuffer + 1));
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY_RAW);
            Wire.write(commandRet);,
        ret)
        DEBUG_PRINT("Received MSG_TYPE_COMMAND with ret=%s\n", commandRet);
        break;

    // If a reply is coming my way, I will relay it to the Serial interface.
    // Same for stream.
    case MSG_TYPE_REPLY:
    case MSG_TYPE_STREAM:
        Serial.printf("%s\n", receivedDataBuffer + 1);
        DEBUG_PRINT("Received MSG_TYPE_REPLY/STREAM\n")
        break; 
    // If a raw reply is coming my way, I will relay it exactly to the Serial interface.
    case MSG_TYPE_REPLY_RAW:
        Serial.printf("%s", receivedDataBuffer + 1);
        DEBUG_PRINT("Received MSG_TYPE_REPLY_RAW\n")
        break;
    case MSG_TYPE_BUFFER:
        Serial.printf(" %f,", *((float *)(receivedDataBuffer + 1)));
        receivingBuffer = true;
        DEBUG_PRINT("Received MSG_TYPE_BUFFER\n")
        break;

    case MSG_TYPE_BUFFER_END:
        Serial.printf(" %f\n", *((float *)(receivedDataBuffer + 1)));
        receivingBuffer = false;
        DEBUG_PRINT("Received MSG_TYPE_BUFFER_END\n")
        break;

    // In case someone has just woken up and I'm id=0, I'll see if we're waiting
    // to start calibration. If yes, reset the counter back to the start.
    case MSG_TYPE_ANNOUNCE_ID:
        network.addNodeToNetwork(receivedDataBuffer[1]);
        if(my_id == 0 && calibrator.waiting()) {
            calibrator.resetWait();
        }
        DEBUG_PRINT("Received MSG_TYPE_ANNOUNCE_ID\n")
        break;

    case MSG_TYPE_BEGIN_CALIBRATION:
        /*Serial.printf("Received begin calibration signal.\n");
        // The maestro ignores its own calls to calibrate
        if(!calibrator.isMaestro()) {
            // Broadcast our id so the highest id can be determined
            SEND_MSG(0, RETRY_TIMEOUT_MS,
                Wire.write(MSG_TYPE_FIND_HIGHEST_ID);
                Wire.write(my_id);,
            ret)
        } else {
            calibrator.setHighestId(my_id);
            calibrator.resetWaitId();
        }*/
        DEBUG_PRINT("Received MSG_TYPE_BEGIN_CALIBRATION\n")
        break;

    case MSG_TYPE_ROLL_CALL:
        network.resetNetwork();
        SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_ANNOUNCE_ID);
            Wire.write(myID);,
        ret)
        DEBUG_PRINT("Received MSG_TYPE_ROLL_CALL\n")
        break;

    case MSG_TYPE_CALIBRATE_ID:
        // The maestro ignores its own calls to calibrate
        if(!calibrator.isMaestro()) {
            if(receivedDataBuffer[1] != my_id)
                calibrator.calibrateGainId(receivedDataBuffer[1]);
            else
                calibrator.selfCalibrate(my_id);
        }
        DEBUG_PRINT("Received MSG_TYPE_CALIBRATE_ID\n")
        break;

    case MSG_TYPE_END_CALIBRATION:
        // The maestro ignores its own calls to calibrate
        calibrator.endCalibration();
        DEBUG_PRINT("Received MSG_TYPE_END_CALIBRATION\n")
        break;
    case MSG_TYPE_VERIFY_LIST:
        if(!network.compareNetwork((uint8_t *)receivedDataBuffer + 2, (uint8_t)receivedDataBuffer[1])) {
            SEND_MSG(0, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_VERIFY_LIST_NACK);,
            ret)
        }
        DEBUG_PRINT("Received MSG_TYPE_VERIFY_LIST\n")
        break;
    case MSG_TYPE_VERIFY_LIST_NACK:
        if(verifyAlarm != -1) {
            cancel_alarm(verifyAlarm);
            waitVerify = false;
        }
        successfulRegister = false;
        DEBUG_PRINT("Received MSG_TYPE_VERIFY_LIST_NACK\n")
        break;
    case MSG_TYPE_CONSENSUS_START:
        consensus.setState(CONSENSUS_STATE_COMPUTING_LOCAL);
        consensus.resetDiMean();
        consensus.resetDi();
        consensus.resetIteration();
        consensus.initLagrangeMultipliers();
        DEBUG_PRINT("Received MSG_TYPE_CONSENSUS_START\n")
        break;
    case MSG_TYPE_CONSENSUS_D:
        DEBUG_PRINT("Received MSG_TYPE_CONSENSUS_D\n")
        byte msg_iter_byte = receivedDataBuffer[1]; // only receives LSB of iteration number
        if(consensus.active() && consensus.notReceived(receivedDataBuffer[0]) && msg_iter_byte == (byte)consensus.iteration) {
            double receivedD[MAX_DEVICES];
            memcpy(receivedD, (const void*) (receivedDataBuffer + 1), sizeof(double) * network.getNumberNodesNetwork());
            for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++) {
                DEBUG_PRINT("d[%hhu] = %lf\n", i, receivedD[i])
            }
            consensus.updateDiMean(receivedD);
            consensus.received(receivedDataBuffer[0]);
        }
        else
            DEBUG_PRINT("Not running consensus\n")
        break;
    case MSG_TYPE_CONSENSUS_ASK_D:
        DEBUG_PRINT("Received MSG_TYPE_CONSENSUS_ASK_D\n")
        if(myID == receivedDataBuffer[1] && (consensus.state == CONSENSUS_STATE_WAITING_FOR_NEIGHBORS ||
            consensus.state == CONSENSUS_STATE_NOT_STARTED)) {
            SEND_MSG(0, RETRY_TIMEOUT_MS,
                Wire.write(MSG_TYPE_CONSENSUS_D);
                Wire.write((byte) consensus.iteration);
                Wire.write((uint8_t*)consensus.di, sizeof(double) * network.getNumberNodesNetwork());,
            ret)
        }
        break;
    case MSG_TYPE_CONSENSUS_CONVERGENCE:
        DEBUG_PRINT("Received MSG_TYPE_CONSENSUS_CONVERGENCE\n")
        if(consensus.active()) {

        }
        else
            DEBUG_PRINT("Not running consensus\n")
        break;
    default:
        DEBUG_PRINT("===========Message wasn't well read. Code %d===========\n", receivedMsg)
        break;
    }
}

void Comms::eventLoop() volatile
{
    flushError();
    processReceivedData();
    streamVars();
}

void Comms::startVerifyAckAlarm() volatile {
    if(verifyAlarm != -1)
        cancel_alarm(verifyAlarm);

    verifyAlarm = add_alarm_in_ms(VERIFY_WAIT_MS, [](long int, void* instance) 
                                  -> long long int {((volatile Comms *)instance)->waitVerify = false; return 0;},(void*)this, false);
}

void Comms::startRollCallAlarm() volatile {
    if(rollCallAlarm != -1)
        cancel_alarm(verifyAlarm);

    rollCallAlarm = add_alarm_in_ms(ROLL_CALL_WAIT_MS, [](long int, void* instance) 
                                  -> long long int {((volatile Comms *)instance)->waitRollCall = false; return 0;}, (void*)this, false);
}

void parseSerial(volatile Comms &comms) {
    char *string = (char *)Serial.readStringUntil('\n').c_str();
    DEBUG_PRINT("Received: '%s'\n", string)
    int ret = comms.processCommand(string);
    (void)ret;
    //DEBUG_PRINT("Process command returned %d\n", ret)
}