#include <Arduino.h>
#include <EEPROM.h>

#include <pico/stdlib.h>
#include <hardware/dma.h>

#include "calibration.hpp"
#include "controller.hpp"
#include "globals.hpp"
#include "interface.hpp"
#include "utilities.hpp"
#include "comms.hpp"
#include "parser.hpp"

// Initialization of global variables

float gammaFactor = 0.f;
float gain = 0.f;
float ambientIlluminance = 0.f;
double tauAscending[10], tauDescending[10];
double luxAscending[10], luxDescending[10];

volatile Controller controller;
repeating_timer timerStruct;

volatile Buffer<float, 60*100> luminanceBuffer;
volatile Buffer<float, 60*100> dutyBuffer;
bool streamLuminanceBuffer = false;
bool streamDutyBuffer = false;
VariableStream streamer;
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
    {'a', "<float>", "reply is number + 2", floatArgCom([](float f){printI2C("a %d %f", myID, f+2)}), NULL},
    {'h', "", "help", noArgCom(help), NULL},
    {'C', "", "calibration utilities", NULL, (Command[]){
        {'a', "", "calibrate gamma and tau, save these parameters to EEPROM and activate them", noArgCom(calibrateAutoCommand), NULL},
        {'c', "<bool: gamma> <bool: tau>", "calibrates gamma and/or tau and saves the calibrated parameters", calibrateCommand, NULL},
        {'s', "<bool: gamma> <bool: tau>", "saves gamma and/or tau from calibrated parameters as final (writes to EEPROM)", saveCalibrationCommand, NULL},
        {'p', "", "print latest calibrated parameters and active parameters", noArgCom(printCalibratedCommand), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'g', "", "get command", NULL, (Command[]){
        {'r', "", "reference", printfCom("reference :)"), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'p', "", "ack", noArgCom([](){ACK}), NULL},
    {'t', "", "time", printfCom("t %d %lu", myID, millis()), NULL},
    {'k', "<int>", "callibrator gain <id> to <id>", intArgCom([](int id) {printI2C("k %d %f", myID, calibrator.getGainId(id)) }), NULL},
    {'r', "", "system reset", noArgCom([](){__NVIC_SystemReset(); ACK}), NULL},
    {'\0', "", "", NULL, NULL}
}
);

Comms comms(parser);

void setup() {
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);
    loadParamsStartup();
    calibrateGain(); // This is only necessary for the first time calibration (to allow for tau/gamma calibration)
    // Initialize Serial protocol
    Serial.begin();
    while(!Serial);
    alarm_pool_init_default();
    DEBUG_PRINT("Gain: %f\n", gain);
    comms.init();
    comms.joinNetwork();

    DEBUG_PRINT("Connected to network as node '%d'\n", myID)
    
    //gammaFactor = 1;

    //calibrator.resetWait();
    while(calibrator.waiting()) {
        comms.eventLoop(); // Run event loop to be able to reset wait whenever new nodes join the network
        DEBUG_PRINT("Waiting for calibration...\n")
        delay(100);
    }
    DEBUG_PRINT("Done waiting. Calibration starting...\n")
    if(comms.my_id == 0)
        comms.calibrateNetwork();

}

void loop() {
    if(Serial.available() > 0)
    {
        parseSerial(comms);
    }
    comms.eventLoop();
}
