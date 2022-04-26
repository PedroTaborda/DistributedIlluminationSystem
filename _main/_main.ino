#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
// #include <hardware/dma.h>
#include <pico/stdlib.h>

#include "calibration.hpp"
#include "comms.hpp"
#include "consensus.hpp"
#include "controller.hpp"
#include "globals.hpp"
#include "network.hpp"
#include "parser.hpp"
#include "utilities.hpp"

// Initialization of global variables

float gammaFactor = 0.f;
float gain = 0.f;
float ambientIlluminance = 0.f;
double tauAscending[10], tauDescending[10];
double luxAscending[10], luxDescending[10];


float outBuffer[outBufferSize];

int outBuffer_i = outBufferSize;  // >= outBufferSize means no transfer to be done

alarm_pool_t* core1AlarmPool;

int myID = 0;

IntFloat intfloat0, intfloat1;

bool acknowledge = false;

CommandParser parser(
    (Command[]){
    {'a', "<int>", "sets the anti-windup state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setAntiWindup(val); ACK}), NULL},
    {'b', "<int>", "sets the feedback state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setFeedback(val); ACK}), NULL},
    {'B', "", "get last minute buffer", NULL, (Command[]){
        {'l', "", "measured lumminance", getIlluminanceCommand, NULL},
        {'d', "", "duty cycle", getDutyBufferCommand, NULL},
        {'\0', "", "", NULL, NULL}}},
    {'C', "", "calibration utilities", NULL, (Command[]){
        {'a', "", "calibrate gamma and tau, save these parameters to EEPROM and activate them", noArgCom(calibrateAutoCommand), NULL},
        {'c', "<bool: gamma> <bool: tau>", "calibrates gamma and/or tau and saves the calibrated parameters", calibrateCommand, NULL},
        {'s', "<bool: gamma> <bool: tau>", "saves gamma and/or tau from calibrated parameters as final (writes to EEPROM)", saveCalibrationCommand, NULL},
        {'p', "", "print latest calibrated parameters and active parameters", noArgCom(printCalibratedCommand), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'d', "<float>", "sets the duty cycle", floatArgCom([](float duty){if(duty < 0 || duty > 1) ERR controller.setDutyCycle(duty); ACK}), NULL},
    {'g', "", "get command", NULL, (Command[]){
        {'a', "", "anti-windup", printfCom("a %d %d\n", myID, controller.getAntiWindup()), NULL},
        {'b', "", "feedback", printfCom("b %d %d\n", myID, controller.getFeedback()), NULL},
        {'c', "", "current energy cost", notImplemented, NULL},
        {'d', "", "duty cycle", printfCom("d %d %f\n", myID, controller.getSample().u), NULL},
        {'e', "", "accumulated energy", printfCom("e %d %f\n", myID, controller.getEnergySpent()), NULL},
        {'f', "", "accumulated flicker error", printfCom("f %d %f\n", myID, controller.getFlickerAccumulator()), NULL},
        {'F', "", "average flicker error", printfCom("f %d %f\n", myID, controller.getFlickerAccumulator() / controller.getSampleNumber()), NULL},
        {'l', "", "measured luminance", printfCom("l %d %f\n", myID, controller.getSample().L), NULL},
        {'L', "", "luminance lower bound", notImplemented, NULL},
        {'o', "", "occupancy", printfCom("o %d %d\n", myID, controller.getOccupancy()), NULL},
        {'O', "", "occupied luminance lower bound", notImplemented, NULL},
        {'p', "", "instantaneous power", printfCom("p %d %f\n", myID, controller.getSample().u), NULL},
        {'r', "", "reference", printfCom("r %d %f\n", myID, controller.getSample().Reference), NULL},
        {'t', "", "time since restart", printfCom("t %d %lu\n", myID, to_ms_since_boot(get_absolute_time()) / 1000), NULL},
        {'U', "", "unoccupied luminance lower bound", notImplemented, NULL},
        {'v', "", "accumulated visibility error", printfCom("v %d %f\n", myID, controller.getVisibilityAccumulator()), NULL},
        {'V', "", "average visiblity error", printfCom("f %d %f\n", myID, controller.getVisibilityAccumulator() / controller.getSampleNumber()), NULL},
        {'w', "", "feedforward", printfCom("w %d %d\n", myID, controller.getFeedforward()), NULL},
        {'x', "", "external luminance", printfCom("x %d %f\n", myID, controller.getSample().L - gain * controller.getSample().u), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'h', "", "help", noArgCom(help), NULL},
    {'k', "<int>", "callibrator gain <id> to <id>", intArgCom([](int id) {printI2C("k %d %f\n", myID, calibrator.getGainId(network.getIndexId(id))) }), NULL},
    {'o', "<int>", "sets the occupancy state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setOccupancy(val); ACK}), NULL},
    {'O', "<float>", "sets the occupied lower bound lumminance", notImplemented, NULL},
    {'p', "", "ack", noArgCom([](){ACK}), NULL},
    {'r', "<float>", "sets the reference", floatArgCom([](float reference){if(reference < 0) ERR controller.setReference(reference); ACK}), NULL},
    {'R', "", "system reset", noArgCom([](){__NVIC_SystemReset(); ACK}), NULL},
    {'s', "", "stream variable", NULL, (Command[]){
        {'l', "", "measured lumminance", noArgCom([](){streamLuminance = !streamLuminance; ACK}), NULL},
        {'d', "", "duty cycle", noArgCom([](){streamDuty = !streamDuty; ACK}), NULL},
        {'j', "", "sample time", noArgCom([](){streamSampleTime = !streamSampleTime; ACK}), NULL},
        {'i', "", "integral error", noArgCom([](){streamIntegralError = !streamIntegralError; ACK}), NULL},
        {'t', "", "tracking error", noArgCom([](){streamTrackingError = !streamTrackingError; ACK}), NULL},
        {'s', "", "simulator", noArgCom([](){streamSimulator = !streamSimulator; ACK}), NULL},
        {'r', "", "reference", noArgCom([](){streamReference = !streamReference; ACK}), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'t', "", "time", printfCom("t %d %lu\n", myID, millis()), NULL},
    {'U', "<float>", "sets the unoccupied lower bound lumminance", notImplemented, NULL},
    {'w', "<int>", "sets the feedforward state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setFeedforward(val); ACK}), NULL},
    {'z', "<float>", "sets the consensus reference on node", floatArgCom([](float reference){consensus.setIlluminanceReference(reference); ACK}), NULL},
    {'\0', "", "", NULL, NULL}
}
);

volatile Comms comms(parser);

void setup() {
    analogReadResolution(12);
    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(DAC_RANGE);
    loadParamsStartup();
    // Initialize Serial protocol
    Serial.begin(BAUD_RATE);
    while(!Serial);
    alarm_pool_init_default();
    DEBUG_PRINT("Gain: %f\n", gain);
    comms.init();
    comms.joinNetwork();

    DEBUG_PRINT("Connected to network as node '%d'\n", myID)

    while(calibrator.waiting()) {
        comms.eventLoop(); // Run event loop to be able to reset wait whenever new nodes join the network
    }
    if(comms.my_id == 0) {
        DEBUG_PRINT("Done waiting. Calibration starting...\n")
        comms.calibrateNetwork();
    }

    consensus.start(network.getNumberNodesNetwork(), network.getIndexId(myID), 1.0f,
                    calibrator.getGains(), calibrator.getExternalLuminance());
    controller.turnControllerOn();
}

void loop() {
    if (!receivingBuffer && Serial.available() > 0) parseSerial(comms);

    comms.eventLoop();

    static unsigned long lastTimeBufferComm = micros();
    unsigned long deltaTimeBufferComm = micros() - lastTimeBufferComm;
    int ret;
    if (outBuffer_i < outBufferSize && deltaTimeBufferComm > 20000) {
        if (outBuffer_i == outBufferSize - 1) {
            // last sample
            SEND_MSG(0, 200, Wire.write(MSG_TYPE_BUFFER_END);
                     Wire.write((byte*)&outBuffer[outBuffer_i], sizeof(outBuffer[0]));, ret);
        } else {
            SEND_MSG(0, 200, Wire.write(MSG_TYPE_BUFFER);
                     Wire.write((byte*)&outBuffer[outBuffer_i], sizeof(outBuffer[0]));, ret);
        }

        outBuffer_i++;
        lastTimeBufferComm = micros();
    }

    if(consensus.active()) {
        if(consensus.state == CONSENSUS_STATE_COMPUTING_LOCAL) {
            double *sol = consensus.optimumSolution();
            consensus.updateDiMean(sol);
            consensus.received(myID);
            //bool seen[network.getNumberNodesNetwork()] = {false};
            //for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++) {
                /*if(seen[i]) continue;
                uint8_t address = addr_offset + network.getNetwork()[i];
                Wire.requestFrom(address, 1);
                while(Wire.available())
                    seen[i] = Wire.read();

                if(seen[i]) {
                    Wire.requestFrom(address, sizeof(double) * network.getNumberNodesNetwork() + 1);
                    double input[network.getNumberNodesNetwork()];
                    uint8_t *readBuffer = (uint8_t *)&input;
                    Wire.read();
                    while(Wire.available()) {
                        *readBuffer = Wire.read();
                        readBuffer++;
                    }
                    for(uint8_t j = 0; j < network.getNumberNodesNetwork(); j++) {
                        DEBUG_PRINT("Local d[%hhu] = %lf\n", j, input[j])
                    }
                    consensus.updateDiMean(input);
                }*/
            /*signed char address = addr_offset + network.getNetwork()[i];
            DEBUG_PRINT("Sending D to %hhd\n", address)
            long int t0 = millis();*/
            for(uint8_t j = 0; j < network.getNumberNodesNetwork(); j++) {
                DEBUG_PRINT("Local d[%hhu] = %lf\n", j, sol[j])
            }
            SEND_MSG(0, CONSENSUS_RETRY_TIMEOUT_MS,
                Wire.write(MSG_TYPE_CONSENSUS_D);
                Wire.write((uint8_t*)sol, sizeof(double) * network.getNumberNodesNetwork());,
            ret)

            //}
        }
        else if(consensus.state == CONSENSUS_STATE_WAITING_FOR_NEIGHBORS && millis() - consensus.beginWaitTime > RETRY_TIMEOUT_MS) {
            consensus.requestMissingD();
        }
        else if(consensus.state == CONSENSUS_STATE_WAITING_CONSENSUS) {
            consensus.finishIter();
        }
    }
}

void setup1() {
    core1AlarmPool = alarm_pool_create(1, 1);
    controller.setup(0.01, 0.05);
}

void loop1() {

}
