#ifndef CONTROLLER2_HPP
#define CONTROLLER2_HPP

#include <Arduino.h>

#include <utility>

#include "buffer.hpp"
#include "simulator.hpp"

struct sample_t {
    // Overload assignment operator for volatile type
    void operator=(volatile sample_t& a) volatile {
        dur = a.dur;
        L = a.L;
        u = a.u;
        IntegralError = a.IntegralError;
        TrackingError = a.TrackingError;
        SimulatorValue = a.SimulatorValue;
        Reference = a.Reference;
        num = a.num;
    }
    unsigned long dur;  // in us
    float L;            // in lux
    float u;
    float IntegralError;
    float TrackingError;
    float SimulatorValue;
    float Reference;

    int num;
};

class Controller {
public:
    // Default Constructor
    Controller();

    // Setup function
    void setup(float proportionalGain, float integralGain, repeating_timer *timerStruct) volatile;

    // Controller loop
    static bool controllerLoop(repeating_timer *timerStruct);

    // User interaction functions
    float getReference() volatile;
    int getOccupancy() volatile;
    void setReference(float reference) volatile;
    //void setDutyCycle(float duty) volatile;
    void setOccupancy(int occupancy) volatile;
    int getSampleNumber() volatile;  // returns number of latest sample, returns negative if none is saved
    volatile sample_t& getSample() volatile;

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
    void handle_requests() volatile;
    void update_outputs() volatile;
    void changeSimulatorReference(float reference) volatile;

    // variables marked with I are not to be written to by controller core during execution
    // they can be changed at any time by users of this class through interface functions

    // Controller state
    float integralError;
    Simulator simulator;
    bool controllerOn;  // I
    bool simulatorOn;   // I
    int occupancy;
    float reference[2];
    float dutyCycle;
    float currentLux;

    // sample buffer of size 2 such that controller can write to one position while the class user
    // reads the other position
    Buffer<volatile sample_t, 2> latest_sample;

    // Controller auxiliary variables
    float trackingError, simulatorValue;
    unsigned long lastTimestamp;
    unsigned long sampleDuration;
    // Controller parameters
    float samplingTime;
    float proportionalGain, integralGain;    // I
    bool antiWindup, feedback, feedforward;  // I

    bool ControlLoopOn;  // true when timer is executing control loop

    // interface variables, all marked with I
    int occupancy_req;
    int refL_mlux;        // in millis of lux
    bool control_on_req;  // false -> request control off; true ->request control on
    // ---------------------------

    // these are written both by controller core and vy interface functions, synchonously
    bool new_ref;
    // ---------------------------
};

// Controller Instance
extern volatile Controller controller;

#endif  // CONTROLLER_HPP