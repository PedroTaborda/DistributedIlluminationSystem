#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <Arduino.h>

// Converts the voltage read at the voltage divider into the corresponding LUX value
float LDRVoltageToLux(float voltage);
// Measures the voltage at the ADC, using a certain number of samples in order
// to filter noise
float measureVoltage(int numberSamples);

// write duty_cycle to DAC, duty is in [0, 1]
void set_u(float duty);
// Checks if two floating point numbers are equal up to some precision
bool fequal(float left, float right);
// Multicore safe expression macro
#define SAFE_M_ACCESS(x)    \
    rp2040.idleOtherCore(); \
    x rp2040.resumeOtherCore();
// Interrupt safe expression macro
#define SAFE_I_ACCESS(x) \
    noInterrupts();      \
    x interrupts();

// Union to simplify passing floats in the inter-core FIFO queues.
typedef union {
    float from;
    uint32_t to;
} IntFloat;

#endif  // UTILITES_HPP