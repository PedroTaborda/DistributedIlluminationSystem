#ifndef CONTROLLER2_HPP
#define CONTROLLER2_HPP

#include <Arduino.h>

#include <utility>

#include "buffer.hpp"
#include "globals.hpp"
#include "interface.hpp"
#include "simulator.hpp"
#include "utilities.hpp"

struct sample_t {
    unsigned long num;
    unsigned long dur;  // in us
    float L;            // in lux
    float u;
    float IntegralError;
    float TrackingError;
    float SimulatorValue;
    float Reference;
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
    void setDutyCycle(float duty) volatile;
    void setOccupancy(int occupancy) volatile;
    int getSampleNumber() volatile;  // returns number of latest sample, returns 0 if none is saved
    sample_t getSample() volatile;

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
    float getDutyCycle() volatile;
    float getIlluminance() volatile;
    float getEnergySpent() volatile;
    float getVisibilityAccumulator() volatile;
    float getFlickerAccumulator() volatile;

    void getDutyBuffer(float out[60 * 100]) volatile;
    void getIlluminanceBuffer(float out[60 * 100]) volatile;

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
    Buffer<sample_t, 2> latest_sample;

    // Controller auxiliary variables
    float trackingError, simulatorValue;
    unsigned long lastTimestamp;
    unsigned long sampleDuration;
    // More than 60 seconds so there is a margin between head and tail of buffer
    Buffer<float, 60 * 100 + 10> luminanceBuffer;
    Buffer<float, 60 * 100 + 10> dutyBuffer;
    double energy;
    double visibilityAccumulator;
    double flickerAccumulator;
    double previousFlicker;
    double previousLux;
    unsigned long sampleNumber;

    // Controller parameters
    float samplingTime;
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

#endif  // CONTROLLER_HPP