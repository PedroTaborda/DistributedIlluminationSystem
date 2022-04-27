#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Arduino.h>
#include <stdint.h>

//  #include <utility> idk why this was here

#include "buffer.hpp"
#include "simulator.hpp"

struct sample_t {
    // Overload assignment operator for volatile type
    void operator=(volatile sample_t a) volatile {
        time = a.time;
        L = a.L;
        u = a.u;
        IntegralError = a.IntegralError;
        TrackingError = a.TrackingError;
        SimulatorValue = a.SimulatorValue;
        Reference = a.Reference;
        num = a.num;
    }
    sample_t() {}
    sample_t(volatile sample_t &a) {
        time = a.time;
        L = a.L;
        u = a.u;
        IntegralError = a.IntegralError;
        TrackingError = a.TrackingError;
        SimulatorValue = a.SimulatorValue;
        Reference = a.Reference;
        num = a.num;
    }
    uint64_t time;  // in us
    float L;            // in lux
    float u;
    float IntegralError;
    float TrackingError;
    float SimulatorValue;
    float Reference;

    int num;
};

#define outBufferSize (60 * 100)
class Controller {
   public:
    // Default Constructor
    Controller();

    // Setup function
    void setup(float proportionalGain, float integralGain) volatile;

    // Controller loop
    static bool controllerLoop(repeating_timer *timerStruct);

    // User interaction functions
    float getReference() volatile;
    float getOccupiedReference() volatile;
    float getUnoccupiedReference() volatile;
    int getOccupancy() volatile;
    void setInnerReference(float reference) volatile;
    void setReference(float reference) volatile;
    void setOccupiedReference(float reference) volatile;
    void setUnoccupiedReference(float reference) volatile;
    void setDutyCycle(float duty) volatile;
    void setDutyCycleFeedforward(float duty) volatile;
    void setOccupancy(int occupancy) volatile;
    int getSampleNumber() volatile;  // returns number of latest sample, returns 0 if none is saved
    sample_t getSample() volatile;

    // Controller settings
    void turnControllerOff() volatile;
    void turnControllerOn() volatile;
    void setAntiWindup(bool antiWindup) volatile;
    void setFeedback(bool feedback) volatile;
    void setFeedforward(bool feedforward) volatile;
    void setSimulator(int simulator) volatile;
    void setProportionalGain(float proportionalGain) volatile;
    void setIntegralGain(float integralGain) volatile;

    bool getAntiWindup() volatile;
    bool getFeedback() volatile;
    bool getFeedforward() volatile;
    float getProportionalGain() volatile;
    float getIntegralGain() volatile;
    float getDutyCycle() volatile;
    float getIlluminance() volatile;
    float getEnergySpent() volatile;
    float getVisibilityAccumulator() volatile;
    float getFlickerAccumulator() volatile;

    void getDutyBuffer(float out[outBufferSize]) volatile;
    void getIlluminanceBuffer(float out[outBufferSize]) volatile;

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
    float innerReference;
    float dutyCycle;
    float feedforwardTerm = 0.f, feedforwardTermReq = 0.f;
    float currentLux;

    // sample buffer of size 2 such that controller can write to one position while the class user
    // reads the other position
    Buffer<volatile sample_t, 2> latest_sample;

    // Controller auxiliary variables
    float trackingError, simulatorValue;
    unsigned long sampleInstant;
    // More than outBufferSize so there is a margin between head and tail of buffer
    Buffer<float, outBufferSize + 10> luminanceBuffer;
    Buffer<float, outBufferSize + 10> dutyBuffer;
    double energy;
    double visibilityAccumulator;
    double flickerAccumulator;
    double previousFlicker;
    double previousLux;
    unsigned long sampleNumber;

    // Controller parameters
    float samplingPeriod;
    float proportionalGain, integralGain;
    bool antiWindup, feedback, feedforward;  // I

    bool ControlLoopOn;  // true when timer is executing control loop

    // interface input variables, all marked with I
    int occupancy_req;
    float proportionalGain_req, integralGain_req;
    int refL_mlux;        // in millis of lux
    bool control_on_req;  // false -> request control off; true ->request control on
    // ---------------------------

    // interface output variables
    float energy_out;
    float visibilityAccumulator_out;
    float flickerAccumulator_out;
    // ----------------------

    // these are written both by controller core and vy interface functions, synchonously
    bool new_ref;
    // ---------------------------
};

// Controller Instance
extern volatile Controller controller;

// Communication functions

char *getDutyBufferCommand(const char *args);

char *getIlluminanceCommand(const char *args);

#endif  // CONTROLLER_HPP