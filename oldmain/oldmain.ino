#include <Arduino.h>
#include <EEPROM.h>
#include <hardware/dma.h>
#include <pico/stdlib.h>

#include "calibration.hpp"
#include "controller2.hpp"
#include "globals.hpp"
#include "interface.hpp"
#include "utilities.hpp"

// Initialization of global variables

float gammaFactor = 0.f;
float gain = 0.f;
float ambientIlluminance = 0.f;
double tauAscending[10], tauDescending[10];
double luxAscending[10], luxDescending[10];

repeating_timer timerStruct;
alarm_pool_t *core1AlarmPool;
volatile Controller controller;
bool streamLuminance = false;
bool streamDuty = false;
bool streamJitter = false;
bool streamIntegralError = false;
bool streamTrackingError = false;
bool streamSimulator = false;
bool streamReference = false;

// CORE 0 is in charge of communications with the computer
// CORE 1 handles the controller

void setup() {
    // Initialize Serial protocol
    Serial.begin();

    // Pause the other core and read constants from the EEPROM
    rp2040.idleOtherCore();

    EEPROM.begin(4096);
    int EEPROMAddress = 0;
    EEPROM.get(EEPROMAddress, gammaFactor);
    EEPROMAddress += sizeof(float);

    for (int tauCounter = 0; tauCounter < 10; tauCounter++) {
        float lux, tau;
        EEPROM.get(EEPROMAddress, lux);
        EEPROMAddress += sizeof(float);
        EEPROM.get(EEPROMAddress, tau);
        EEPROMAddress += sizeof(float);

        luxAscending[tauCounter] = lux;
        tauAscending[tauCounter] = tau;
    }

    for (int tauCounter = 0; tauCounter < 10; tauCounter++) {
        float lux, tau;
        EEPROM.get(EEPROMAddress, lux);
        EEPROMAddress += sizeof(float);
        EEPROM.get(EEPROMAddress, tau);
        EEPROMAddress += sizeof(float);

        luxDescending[tauCounter] = lux;
        tauDescending[tauCounter] = tau;
    }

    EEPROM.end();
    // Resume the other core
    rp2040.resumeOtherCore();
}

void loop() {
    if (Serial.available() > 0) {
        parseSerial();
    }

    streamVariables();

    // uint32_t queueResult;
    // if (rp2040.fifo.pop_nb(&queueResult)) {
    //     char command = (char)queueResult;
    //     char variable;
    //     float variableValue;
    //     unsigned long timeStamp;
    //     switch (command) {
    //         case 'B':
    //             queueResult = rp2040.fifo.pop();
    //             variable = (char)queueResult;
    //             Serial.print("B ");
    //             Serial.print(variable);
    //             Serial.print(" 0 ");
    //             for (int i = 0; i < 60 * 100; i++) {
    //                 Serial.print(copyBuffer[(currentHead + i) % (60 * 100)], 6);
    //                 i == 60 * 100 - 1 ? Serial.print('\n') : Serial.print(',');
    //             }
    //             bufferLock = false;
    //             streamDutyBuffer = false;
    //             streamLuminanceBuffer = false;
    //             break;
    //         case 's':
    //             queueResult = rp2040.fifo.pop();
    //             variable = (char)queueResult;
    //             queueResult = rp2040.fifo.pop();
    //             intfloat0.to = queueResult;
    //             variableValue = intfloat0.from;
    //             queueResult = rp2040.fifo.pop();
    //             timeStamp = (unsigned long)queueResult;

    //             Serial.print("s ");
    //             Serial.print(variable);
    //             Serial.print(" 0 ");
    //             Serial.print(variableValue, 6);
    //             Serial.print(' ');
    //             Serial.println(timeStamp / 1000);
    //             break;
    //     }
    // }
}

void setup1() {
    // Initialize the ADC and DAC settings
    analogReadResolution(12);
    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(DAC_RANGE);

    // Calibrate the system's static gain
    calibrateGain();

    core1AlarmPool = alarm_pool_create(1, 1);

    // Setup the controller
    controller.setup(0.01, 0.05, &timerStruct);
}

void loop1() {}