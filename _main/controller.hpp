#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <utility>

#include <Arduino.h>

#include "buffer.hpp"
#include "simulator.hpp"

class VariableStream;

class Controller {
public:

    // Default Constructor
    Controller();

    // Setup function
    void setup(float proportionalGain, float integralGain, repeating_timer *timerStruct) volatile;

    // Controller loop
    static bool controllerLoop(repeating_timer *timerStruct);

    // User interaction functions
    void changeSimulatorReference(float reference) volatile;
    float getReference() volatile;
    int getOccupancy() volatile;
    void setReference(float reference) volatile;
    void setDutyCycle(float duty) volatile;
    void setOccupancy(int occupancy) volatile;

    // Controller settings
    void turnControllerOff() volatile;
    void turnControllerOn() volatile;
    void toggleAntiWindup() volatile;
    void toggleFeedback() volatile;
    void toggleFeedforward() volatile;
    void setSimulator(int simulator) volatile;
    void setProportionalGain(float proportionalGain) volatile;
    void setIntegralGain(float integralGain) volatile;

    bool getAntiWindup() volatile;
    bool getFeedback() volatile;
    bool getFeedforward() volatile;
    float getProportionalGain() volatile;
    float getIntegralGain() volatile;

private:

    friend VariableStream;

    // Controller state
    float integralError;
    Simulator simulator;
    bool controllerOn;
    bool simulatorOn;
    int occupancy;
    float reference[2];
    float dutyCycle[2];

    // Controller auxiliary variables
    float trackingError, simulatorValue;
    unsigned long lastTimestamp;
    long jitter;

    // Controller parameters
    float proportionalGain, integralGain;
    float samplingTime;
    bool antiWindup, feedback, feedforward;
};

#endif //CONTROLLER_HPP