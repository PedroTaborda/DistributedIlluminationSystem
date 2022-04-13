#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <Arduino.h>

#include "controller.hpp"

// Constant values for configuring the micro controller
const int BAUD_RATE = 115200;
const int DAC_RANGE = 4096;
const int LED_PIN = 15;
const int LDR_PIN = A0;
const int PWM_FREQUENCY = 60000;
const int MAX_VOLTAGE_SAMPLES = 100;
const float FLOAT_RELATIVE_TOLERANCE = 0.001f;

// System parameters
extern float gammaFactor;
extern double tauAscending[10], tauDescending[10];
extern double luxAscending[10], luxDescending[10];
extern float gain;
extern float ambientIlluminance;

// Controller Instance
extern volatile Controller controller;

// Buffers for variables
extern volatile Buffer<float, 60*100> luminanceBuffer;
extern volatile Buffer<float, 60*100> dutyBuffer;
extern bool streamLuminanceBuffer;
extern bool streamDutyBuffer;
extern VariableStream streamer;
extern volatile double energy;
extern volatile double visibilityAccumulator;
extern volatile double flickerAccumulator;
extern volatile double previousFlicker;
extern volatile double previousLux;
extern volatile unsigned long sampleNumber;

// Alarm Pool object to allow interrupts on the second core
extern alarm_pool_t* core1AlarmPool;

// Copy buffer and related variables. This is used to transfer a
// buffer from core 1 to core 0, bypassing the FIFO comunication queue
// between cores, given the information volume.
extern bool bufferLock;
extern int dmaChannel;
extern float copyBuffer[60*100];
extern int currentHead;

#endif //GLOBALS_HPP