#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <Arduino.h>

#include "controller2.hpp"

// Constant values for configuring the micro controller
const int BAUD_RATE = 115200;
const int DAC_RANGE = 4096;
const int LED_PIN = 13;
const int LDR_PIN = A0;
const int PWM_FREQUENCY = 60000;
const int MAX_VOLTAGE_SAMPLES = 100;
const float FLOAT_RELATIVE_TOLERANCE = 0.001f;

// System parameters
#define SAMPLE_PERIOD_US (10000)
extern float gammaFactor;
extern double tauAscending[10], tauDescending[10];
extern double luxAscending[10], luxDescending[10];
extern float gain;
extern float ambientIlluminance;

// Controller Instance
extern volatile Controller controller;

extern bool streamLuminance;
extern bool streamDuty;
extern bool streamJitter;
extern bool streamIntegralError;
extern bool streamTrackingError;
extern bool streamSimulator;
extern bool streamReference;

// Alarm Pool object to allow interrupts on the second core
extern alarm_pool_t* core1AlarmPool;

#endif  // GLOBALS_HPP