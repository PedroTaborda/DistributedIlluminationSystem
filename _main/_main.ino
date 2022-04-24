#include <Arduino.h>
#include <EEPROM.h>

#include <pico/stdlib.h>
#include <hardware/dma.h>

#include "calibration.hpp"
#include "controller.hpp"
#include "globals.hpp"
#include "utilities.hpp"
#include "comms.hpp"
#include "parser.hpp"

// Initialization of global variables

float gammaFactor = 0.f;
float gain = 0.f;
float ambientIlluminance = 0.f;
double tauAscending[10], tauDescending[10];
double luxAscending[10], luxDescending[10];

volatile Buffer<volatile float, 60*100> luminanceBuffer;
volatile Buffer<volatile float, 60*100> dutyBuffer;
bool streamLuminanceBuffer = false;
bool streamDutyBuffer = false;
volatile unsigned long lastTimestamp = 0;
volatile double energy = 0.f;
volatile double visibilityAccumulator = 0.f;
volatile double flickerAccumulator = 0.f;
volatile double previousFlicker = 0.f;
volatile double previousLux = 0.f;
volatile unsigned long sampleNumber = 0;

alarm_pool_t* core1AlarmPool;

bool bufferLock = false;
int dmaChannel;
float copyBuffer[60*100];
int currentHead = 0;

int myID = 0;

IntFloat intfloat0, intfloat1;

CommandParser parser(
    (Command[]){
    {'a', "<int>", "sets the anti-windup state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setOccupancy(val); ACK}), NULL},
    {'b', "<int>", "sets the feedback state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setFeedback(val); ACK}), NULL},
    {'B', "", "get last minute buffer", NULL, (Command[]){
        {'l', "", "measured lumminance", notImplemented, NULL},
        {'d', "", "duty cycle", notImplemented, NULL},
        {'\0', "", "", NULL, NULL}}},
    {'C', "", "calibration utilities", NULL, (Command[]){
        {'a', "", "calibrate gamma and tau, save these parameters to EEPROM and activate them", noArgCom(calibrateAutoCommand), NULL},
        {'c', "<bool: gamma> <bool: tau>", "calibrates gamma and/or tau and saves the calibrated parameters", calibrateCommand, NULL},
        {'s', "<bool: gamma> <bool: tau>", "saves gamma and/or tau from calibrated parameters as final (writes to EEPROM)", saveCalibrationCommand, NULL},
        {'p', "", "print latest calibrated parameters and active parameters", noArgCom(printCalibratedCommand), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'d', "<float>", "sets the duty cycle", floatArgCom([](float duty){if(duty < 0 || duty > 1) ERR controller.turnControllerOff(); set_u(duty); ACK}), NULL},
    {'g', "", "get command", NULL, (Command[]){
        {'a', "", "anti-windup", printfCom("a %d %d", myID, controller.getAntiWindup()), NULL},
        {'b', "", "feedback", printfCom("b %d %d", myID, controller.getFeedback()), NULL},
        {'c', "", "current energy cost", notImplemented, NULL},
        {'d', "", "duty cycle", printfCom("d %d %f", myID, controller.getSample().u), NULL},
        {'e', "", "accumulated energy", printfCom("e %d %f", myID, energy), NULL},
        {'f', "", "accumulated flicker error", printfCom("f %d %f", myID, flickerAccumulator), NULL},
        {'F', "", "average flicker error", printfCom("f %d %f", myID, flickerAccumulator / sampleNumber), NULL},
        {'l', "", "measured luminance", printfCom("l %d %f", myID, controller.getSample().L), NULL},
        {'L', "", "luminance lower bound", notImplemented, NULL},
        {'o', "", "occupancy", printfCom("o %d %d", myID, controller.getOccupancy()), NULL},
        {'O', "", "occupied luminance lower bound", notImplemented, NULL},
        {'p', "", "instantaneous power", printfCom("p %d %f", myID, controller.getSample().u), NULL},
        {'r', "", "reference", printfCom("r %d %f", myID, controller.getSample().Reference), NULL},
        {'t', "", "time since restart", printfCom("t %d %lu", myID, to_ms_since_boot(get_absolute_time()) / 1000), NULL},
        {'U', "", "unoccupied luminance lower bound", notImplemented, NULL},
        {'v', "", "accumulated visibility error", printfCom("v %d %f", myID, visibilityAccumulator), NULL},
        {'V', "", "average visiblity error", printfCom("f %d %f", myID, visibilityAccumulator / sampleNumber), NULL},
        {'w', "", "feedforward", printfCom("w %d %d", myID, controller.getFeedforward()), NULL},
        {'x', "", "external luminance", printfCom("x %d %f", myID, controller.getSample().L - gain * controller.getSample().u), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'h', "", "help", noArgCom(help), NULL},
    {'k', "<int>", "callibrator gain <id> to <id>", intArgCom([](int id) {printI2C("k %d %f", myID, calibrator.getGainId(id)) }), NULL},
    {'o', "<int>", "sets the occupancy state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setOccupancy(val); ACK}), NULL},
    {'O', "<float>", "sets the occupied lower bound lumminance", notImplemented, NULL},
    {'p', "", "ack", noArgCom([](){ACK}), NULL},
    {'r', "<float>", "sets the reference", floatArgCom([](float reference){if(reference < 0) ERR controller.setReference(reference); ACK}), NULL},
    {'R', "", "system reset", noArgCom([](){__NVIC_SystemReset(); ACK}), NULL},
    {'s', "", "stream variable", NULL, (Command[]){
        {'l', "", "measured lumminance", notImplemented, NULL},
        {'d', "", "duty cycle", notImplemented, NULL},
        {'\0', "", "", NULL, NULL}}},
    {'t', "", "time", printfCom("t %d %lu", myID, millis()), NULL},
    {'U', "<float>", "sets the unoccupied lower bound lumminance", notImplemented, NULL},
    {'w', "<int>", "sets the feedforward state", intArgCom([](int val){if(val < 0 || val > 1) ERR controller.setFeedforward(val); ACK}), NULL},
    {'\0', "", "", NULL, NULL}
}
);

volatile Comms comms(parser);

void setup() {
    analogReadResolution(12);
    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(DAC_RANGE);
    loadParamsStartup();
    //calibrateGain(); // This is only necessary for the first time calibration (to allow for tau/gamma calibration)
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
    DEBUG_PRINT("Done waiting. Calibration starting...\n")
    if(comms.my_id == 0)
        comms.calibrateNetwork();
}

void loop() {
    if(Serial.available() > 0)
        parseSerial(comms);

    comms.eventLoop();
}

void setup1() {
    core1AlarmPool = alarm_pool_create(1, 1);
    controller.setup(0.01, 0.05);
}

void loop1() {

}
