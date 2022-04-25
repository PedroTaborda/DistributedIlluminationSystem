#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <Arduino.h>

#include "buffer.hpp"

#ifdef DEBUG
#define DEBUG_PRINT(...) {Serial.print("[DEBUG] ");Serial.printf(__VA_ARGS__);}
#else
#define DEBUG_PRINT(...) {}
#endif

const int tauN = 10;
struct luminaireParams{
    float gammaFactor = 1.0;
    float tauAscending[tauN];
    float tauDescending[tauN];
};

// Constant values for configuring the micro controller
const int BAUD_RATE = 115200;
const int DAC_RANGE = 4096;
const int PWM_FREQUENCY = 60000;
const int MAX_VOLTAGE_SAMPLES = 100;
const float FLOAT_RELATIVE_TOLERANCE = 0.001f;

#ifdef ZE
const int LED_PIN = 15;
const int LDR_PIN = A0;
#else
const int LED_PIN = 13;
const int LDR_PIN = A0;
#endif

// System parameters
extern float gammaFactor;
extern double tauAscending[10], tauDescending[10];
extern double voltageAscending[10], voltageDescending[10];
extern float gain;
extern float ambientIlluminance;

// Alarm Pool object to allow interrupts on the second core
extern alarm_pool_t* core1AlarmPool;

extern luminaireParams latestCalibration;
luminaireParams activeParams();

#endif //GLOBALS_HPP