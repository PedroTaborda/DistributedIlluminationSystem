#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
// #include <hardware/dma.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>

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
repeating_timer networkTimer;

int myID = 0;
extern volatile Comms comms;

char *resetIn300ms()
{
    add_alarm_in_ms(
        300, [](long int, void *) -> long long int
        { __NVIC_SystemReset(); },
        NULL, true);
    ACK
}

void recallibrateNetwork() {
    controller.turnControllerOff();
    comms.calibrateNetwork();
    consensus.start(network.getNumberNodesNetwork(), network.getIndexId(myID), 1.0f,
                calibrator.getGains(), ambientIlluminance);
    controller.turnControllerOn();
}

char* printStaticGains() {
    static char buffer[256];
    buffer[0] = '\0';
    sprintf(buffer + strlen(buffer), "k %d", myID);
    for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++)
        sprintf(buffer + strlen(buffer), " %f", calibrator.getGainId(i));
    sprintf(buffer + strlen(buffer), "\n");
    return buffer;
}

CommandParser parser(
    (Command[]){
    {'a', "<int>", "sets the anti-windup state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setAntiWindup(val); ACK}), NULL},
    {'b', "<int>", "sets the feedback state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setFeedback(val); ACK}), NULL},
    {'B', "", "get last minute buffer", NULL, (Command[]){
        {'l', "", "measured lumminance", getIlluminanceCommand, NULL},
        {'d', "", "duty cycle", getDutyBufferCommand, NULL},
        {'\0', "", "", NULL, NULL}}},
    {'c', "<float>", "set cost", floatArgCom([](float cost){consensus.setLocalCost(cost); consensus.setIlluminanceReference(controller.getReference()); ACK}), NULL},
    {'C', "", "calibration utilities", NULL, (Command[]){
        {'a', "", "calibrate gamma and tau, save these parameters to EEPROM and activate them", noArgCom(calibrateAutoCommand), NULL},
        {'c', "<bool: gamma> <bool: tau>", "calibrates gamma and/or tau and saves the calibrated parameters", calibrateCommand, NULL},
        {'k', "<int>", "static gain of <id> to <id>", intArgCom([](int id) {printI2C("k %d %d %f\n", myID, id, calibrator.getGainId(network.getIndexId(id))) }), NULL},
        {'K', "", "static gains to <id>", noArgCom(printStaticGains), NULL},
        {'s', "<bool: gamma> <bool: tau>", "saves gamma and/or tau from calibrated parameters as final (writes to EEPROM)", saveCalibrationCommand, NULL},
        {'S', "", "recallibrates the static gains", noArgCom([]() { ACK}), NULL},
        {'p', "", "print latest calibrated parameters and active parameters", noArgCom(printCalibratedCommand), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'d', "<float>", "sets the duty cycle", floatArgCom([](float duty){if(duty < 0 || duty > 1) ERR controller.setDutyCycle(duty); ACK}), NULL},
    {'g', "", "get command", NULL, (Command[]){
        {'a', "", "anti-windup", printfCom("a %d %d\n", myID, controller.getAntiWindup()), NULL},
        {'b', "", "feedback", printfCom("b %d %d\n", myID, controller.getFeedback()), NULL},
        {'c', "", "current energy cost", printfCom("c %d %f\n", myID, consensus.localCost), NULL},
        {'d', "", "duty cycle", printfCom("d %d %f\n", myID, controller.getSample().u), NULL},
        {'e', "", "accumulated energy", printfCom("e %d %f\n", myID, controller.getEnergySpent()), NULL},
        {'f', "", "accumulated flicker error", printfCom("f %d %f\n", myID, controller.getFlickerAccumulator()), NULL},
        {'F', "", "average flicker error", printfCom("f %d %f\n", myID, controller.getFlickerAccumulator() / controller.getSampleNumber()), NULL},
        {'l', "", "measured luminance", printfCom("l %d %f\n", myID, controller.getSample().L), NULL},
        {'L', "", "luminance lower bound", printfCom("L %d %f\n", myID, controller.getReference()), NULL},
        {'o', "", "occupancy", printfCom("o %d %d\n", myID, controller.getOccupancy()), NULL},
        {'O', "", "occupied luminance lower bound", printfCom("O %d %f\n", myID, controller.getOccupiedReference()), NULL},
        {'p', "", "instantaneous power", printfCom("p %d %f\n", myID, controller.getSample().u), NULL},
        {'r', "", "reference", printfCom("r %d %f\n", myID, controller.getSample().Reference), NULL},
        {'t', "", "time since restart", printfCom("t %d %lu\n", myID, to_ms_since_boot(get_absolute_time()) / 1000), NULL},
        {'U', "", "unoccupied luminance lower bound", printfCom("U %d %f\n", myID, controller.getUnoccupiedReference()), NULL},
        {'v', "", "accumulated visibility error", printfCom("v %d %f\n", myID, controller.getVisibilityAccumulator()), NULL},
        {'V', "", "average visiblity error", printfCom("f %d %f\n", myID, controller.getVisibilityAccumulator() / controller.getSampleNumber()), NULL},
        {'w', "", "feedforward", printfCom("w %d %d\n", myID, controller.getFeedforward()), NULL},
        {'x', "", "external luminance", printfCom("x %d %f\n", myID, controller.getSample().L - gain * controller.getSample().u), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'h', "", "help", noArgCom(help), NULL},
    {'m', "<bool>", "set simulator", boolArgCom([](bool newSimState){controller.setSimulator(newSimState); ACK}), NULL},
    {'o', "<int>", "sets the occupancy state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setOccupancy(val); ACK}), NULL},
    {'O', "<float>", "sets the occupied lower bound lumminance", floatArgCom([](float val){if(val < 0) ERR controller.setOccupiedReference(val); ACK}), NULL},
    {'p', "", "ack", noArgCom([](){ACK}), NULL},
    {'r', "<float>", "sets the reference", floatArgCom([](float reference){if(reference < 0) ERR controller.setReference(reference); ACK}), NULL},
    {'R', "", "system reset", noArgCom(resetIn300ms), NULL},
    {'s', "", "stream variable", NULL, (Command[]){
        {'l', "", "measured lumminance", idArgCom([](uint8_t displayer){streamLuminance = !streamLuminance; luminanceStreamDisplayer = displayer; ACK}), NULL},
        {'d', "", "duty cycle", idArgCom([](uint8_t displayer){streamDuty = !streamDuty; dutyStreamDisplayer = displayer; ACK}), NULL},
        {'i', "", "integral error", idArgCom([](uint8_t displayer){streamIntegralError = !streamIntegralError; integralErrorStreamDisplayer = displayer; ACK}), NULL},
        {'t', "", "tracking error", idArgCom([](uint8_t displayer){streamTrackingError = !streamTrackingError; trackingErrorStreamDisplayer = displayer; ACK}), NULL},
        {'s', "", "simulator", idArgCom([](uint8_t displayer){streamSimulator = !streamSimulator; simulatorStreamDisplayer = displayer; ACK}), NULL},
        {'r', "", "reference", idArgCom([](uint8_t displayer){streamReference = !streamReference; referenceStreamDisplayer = displayer; ACK}), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'S', "<int: N>", "stream skips N samples", intArgCom([](int N){if(N < 0) {ERR} skipSamplesStream = (unsigned int) N; ACK}), NULL},
    {'t', "", "time", printfCom("t %d %lu\n", myID, millis()), NULL},
    {'U', "<float>", "sets the unoccupied lower bound lumminance", floatArgCom([](float val){if(val < 0) ERR controller.setUnoccupiedReference(val); ACK}), NULL},
    {'w', "<int>", "sets the feedforward state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setFeedforward(val); ACK}), NULL},
    {'z', "<float>", "sets the consensus reference on node", floatArgCom([](float reference){consensus.setIlluminanceReference(reference); ACK}), NULL},
    {'\0', "", "", NULL, NULL}
}
);

volatile Comms comms(parser);

double gains1[] = {200, 50, 50};
double gains2[] = {50, 200, 50};
double gains3[] = {50, 50, 200};

double exter1 = 20;
double exter2 = 30;
double exter3 = 40;

void setup() {
    pico_unique_board_id_t uniqueId;
    pico_get_unique_board_id(&uniqueId);

    // Have to use the lower 32 bits. The higher ones lead to the same seed for some reason.
    randomSeed(*(unsigned long *)(&(uniqueId.id[4])));
    analogReadResolution(12);
    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(DAC_RANGE);
    loadParamsStartup();
    // Initialize Serial protocol
    #ifdef DEBUG
    while(!Serial) ;
    #endif
    Serial.begin(BAUD_RATE);
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
                calibrator.getGains(), ambientIlluminance);
    controller.turnControllerOn();
    network.beginAliveCheck(&networkTimer);
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
            
            for(uint8_t j = 0; j < network.getNumberNodesNetwork(); j++) {
                DEBUG_PRINT("Local d[%hhu] = %lf\n", j, sol[j])
            }
            SEND_MSG(0, CONSENSUS_RETRY_TIMEOUT_MS,
                Wire.write(MSG_TYPE_CONSENSUS_D);
                Wire.write((byte) consensus.iteration);
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

    if(network.shouldEmmitAlive())
        network.emmitAliveMessage();
}

void setup1() {
    core1AlarmPool = alarm_pool_create(1, 1);
    controller.setup(0.01, 0.05);

    while(!Serial)
        DEBUG_PRINT("Kp: %f and Ki: %f\n", controller.getProportionalGain(), controller.getIntegralGain())
}

void loop1() {

}
