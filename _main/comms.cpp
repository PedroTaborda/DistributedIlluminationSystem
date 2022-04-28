#include "buffer.hpp"
#include "calibration.hpp"
#include "comms.hpp"
#include "consensus.hpp"
#include "globals.hpp"
#include "network.hpp"
#include "parser.hpp"
#include <Wire.h>

#define SEND_STREAM_MSG(addr, cmdChar, varFloat, tUnsignedLong, ret)          \
    {                                                                         \
        SEND_MSG(                                                             \
            addr, RETRY_TIMEOUT_MS,                                           \
            Wire.write(MSG_TYPE_STREAM);                                      \
            Wire.write(cmdChar);                                              \
            Wire.write((uint8_t *)&varFloat, sizeof(float));                  \
            Wire.write((uint8_t *)&tUnsignedLong, sizeof(unsigned long));,    \
                                                                         ret) \
    }

volatile Comms* _comms;
int asked = 1;

bool receivingBuffer = false;

unsigned int skipSamplesStream = 4;

bool streamLuminance = false;
int luminanceStreamDisplayer = 0;

bool streamDuty = false;
int dutyStreamDisplayer = 0;

bool streamIntegralError = false;
int integralErrorStreamDisplayer = 0;

bool streamTrackingError = false;
int trackingErrorStreamDisplayer = 0;

bool streamSimulator = false;
int simulatorStreamDisplayer = 0;

bool streamReference = false;
int referenceStreamDisplayer = 0;

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
        SEND_MSG_HOLD(my_potential_addr, RETRY_TIMEOUT_MS, ,ret)
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
                DEBUG_PRINT("%hhuth member of the network is %hhu\n", i + 1, network.getNetwork()[i])
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
    int ret;
    // Become the maestro
    calibrator.becomeMaestro();
    DEBUG_PRINT("Calibration starting.\n");
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

    if (luminaireID == myID)
    {
        commandRet = parser.executeCommand(command, myID);
        Serial.println(commandRet == NULL ? "NULL" : commandRet);
        return commandRet == NULL ? PROCESSING_LOCAL_EXECUTION_FAILURE : PROCESSING_OK;
    }

    const char *strippedCommand = command;
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
    
    for (int buf_idx = 1; buf_idx < bytesReceived-1; buf_idx++)
    {
        receivedMsgDataBuffer[dataBufferHead][buf_idx] = (uint8_t)Wire1.read();
    }
    receivedMsgDataBuffer[dataBufferHead][bytesReceived-1] = '\0';
    dataBufferHead = (MSG_BUFFER_SIZE + dataBufferHead + 1) % MSG_BUFFER_SIZE;
    dataBufferItems = min(MSG_BUFFER_SIZE, dataBufferItems + 1);
    receivedDataSize = bytesReceived;
}

void Comms::onRequest() volatile
{
}

void Comms::flushError() volatile{
    if (error)
        Serial.printf("%s\n", errorMsg);
    error = false;
}

void Comms::streamVars() volatile{
    int sampleNum = controller.getSampleNumber();

    if (sampleNum == 0)
        return;
    if (sampleNum <= lastSampleStreamed + (int)skipSamplesStream)
        return;

    lastSampleStreamed = sampleNum;

    sample_t newSample = controller.getSample();
    int ret=0;


    float var = 0.0;
    unsigned long t = 0;
    unsigned char cmd = '_';
    if(streamDuty){
        DEBUG_PRINT("Streaming duty cycle to %d [s d %d %.4f %llu]\n", dutyStreamDisplayer, myID, newSample.u, newSample.time / 1000);
        if (dutyStreamDisplayer == myID)
            Serial.printf("s d %d %.4f %llu\n", myID, newSample.u, newSample.time / 1000);
        else{
            var = (float) newSample.u;
            t = (unsigned long)(newSample.time/1000);
            cmd = 'd';
            SEND_STREAM_MSG(dutyStreamDisplayer + addr_offset, cmd, var, t, ret)
        }
    }
    if(streamLuminance){
        DEBUG_PRINT("Streaming luminance to %d [s l %d %.4f %llu]\n", luminanceStreamDisplayer, myID, newSample.L, newSample.time / 1000);
        if (luminanceStreamDisplayer == myID)
            Serial.printf("s l %d %.4f %llu\n", myID, newSample.L, newSample.time / 1000);
        else{
            var = (float) newSample.L;
            t = (unsigned long)(newSample.time/1000);
            cmd = 'l';
            SEND_STREAM_MSG(luminanceStreamDisplayer + addr_offset, cmd, var, t, ret)
        }
    }
    if(streamIntegralError){
        DEBUG_PRINT("Streaming integral error to %d [s i %d %.4f %llu]\n", integralErrorStreamDisplayer, myID, newSample.IntegralError, newSample.time / 1000);
        if (integralErrorStreamDisplayer == myID)
            Serial.printf("s i %d %.4f %llu\n", myID, newSample.IntegralError, newSample.time / 1000);
        else{
            var = (float) newSample.IntegralError;
            t = (unsigned long)(newSample.time/1000);
            cmd = 'i';
            SEND_STREAM_MSG(integralErrorStreamDisplayer + addr_offset, cmd, var, t, ret)
        }
    }
    if(streamTrackingError){
        DEBUG_PRINT("Streaming tracking error to %d [s t %d %.4f %llu]\n", trackingErrorStreamDisplayer, myID, newSample.TrackingError, newSample.time / 1000);
        if (trackingErrorStreamDisplayer == myID)
            Serial.printf("s t %d %.4f %llu\n", myID, newSample.TrackingError, newSample.time / 1000);
        else{
            var = (float) newSample.TrackingError;
            t = (unsigned long)(newSample.time/1000);
            cmd = 't';
            SEND_STREAM_MSG(trackingErrorStreamDisplayer + addr_offset, cmd, var, t, ret)
        }
    }
    if(streamSimulator){
        DEBUG_PRINT("Streaming simulator to %d [s s %d %.4f %llu]\n", simulatorStreamDisplayer, myID, newSample.SimulatorValue, newSample.time / 1000);
        if (simulatorStreamDisplayer == myID)
            Serial.printf("s s %d %.4f %llu\n", myID, newSample.SimulatorValue, newSample.time / 1000);
        else{
            var = (float) newSample.SimulatorValue;
            t = (unsigned long)(newSample.time/1000);
            cmd = 's';
            SEND_STREAM_MSG(simulatorStreamDisplayer + addr_offset, cmd, var, t, ret)
        }
    }
    if(streamReference){
        DEBUG_PRINT("Streaming reference to %d [s r %d %.4f %llu]\n", referenceStreamDisplayer, myID, newSample.Reference, newSample.time / 1000);
        if (referenceStreamDisplayer == myID)
            Serial.printf("s r %d %.4f %llu\n", myID, newSample.Reference, newSample.time / 1000);
        else{
            var = (float) newSample.Reference;
            t = (unsigned long)(newSample.time/1000);
            cmd = 'r';
            SEND_STREAM_MSG(referenceStreamDisplayer + addr_offset, cmd, var, t, ret)
        }
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

    volatile uint8_t *receivedDataBuffer = NULL;
    if(dataBufferItems != 0)
        receivedDataBuffer = receivedMsgDataBuffer[(MSG_BUFFER_SIZE + dataBufferHead - dataBufferItems--) % MSG_BUFFER_SIZE];
    interrupts();

    if (receivedMsg == MSG_TYPE_NONE)
        return;

    processedMessageCounter += 1;
    DEBUG_PRINT("Received message = %li, Processed received message = %li\n", messageCounter, processedMessageCounter)

    int ret;
    char *commandRet = NULL;
    
    uint8_t streamer = 0;
    unsigned char cmd = '_';
    float var = 0.0;
    unsigned long t = 0;

    switch (receivedMsg)
    {
    // If a command was issued to me, I will execute it and reply with the result.
    case MSG_TYPE_COMMAND:
        commandRet = parser.executeCommand((const char *)(receivedDataBuffer + 1), receivedDataBuffer[0]);
        SEND_MSG(receivedDataBuffer[0]+addr_offset, RETRY_TIMEOUT_MS,
            Wire.write(MSG_TYPE_REPLY_RAW);
            Wire.write(commandRet);,
        ret)
        DEBUG_PRINT("Received MSG_TYPE_COMMAND with ret=%s\n", commandRet);
        break;

    // If a reply is coming my way, I will relay it to the Serial interface.
    case MSG_TYPE_REPLY:
        Serial.printf("%s\n", receivedDataBuffer + 1);
        DEBUG_PRINT("Received MSG_TYPE_REPLY/STREAM\n")
        break; 
    // Unpack stream message and relay it to the Serial interface.
    case MSG_TYPE_STREAM:
        streamer = receivedDataBuffer[0];
        cmd = receivedDataBuffer[1];
        var = *((float *)(receivedDataBuffer + 2));
        t = *((unsigned long *)(receivedDataBuffer + 2 + sizeof(float)));
        Serial.printf("s %c %d %.4f %lu\n", cmd, streamer, var, t);
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
        if(consensus.active() && consensus.notReceived(receivedDataBuffer[0]) && receivedDataBuffer[1] == (byte)consensus.iteration) {
            double receivedD[MAX_DEVICES];
            memcpy(receivedD, (const void*) (receivedDataBuffer + 2), sizeof(double) * network.getNumberNodesNetwork());
            bool badValues = false;
            for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++) {
                DEBUG_PRINT("d[%hhu] = %lf\n", i, receivedD[i])
                if(fabs(receivedD[i]) > 1000.f)
                    badValues = true;
            }
            if(!badValues) {
                consensus.updateDiMean(receivedD);
                consensus.received(receivedDataBuffer[0]);
            }
        }
        else
            DEBUG_PRINT("Not running consensus\n")
        break;
    case MSG_TYPE_CONSENSUS_ASK_D:
        DEBUG_PRINT("Received MSG_TYPE_CONSENSUS_ASK_D\n")
        if(myID == receivedDataBuffer[1] && consensus.state != CONSENSUS_STATE_COMPUTING_LOCAL && receivedDataBuffer[2] <= consensus.iteration) {
            SEND_MSG(0, RETRY_TIMEOUT_MS,
                Wire.write(MSG_TYPE_CONSENSUS_D);
                Wire.write((byte) receivedDataBuffer[2]);
                Wire.write((uint8_t*)consensus.getIterationSolution(receivedDataBuffer[2]), sizeof(double) * network.getNumberNodesNetwork());,
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
}