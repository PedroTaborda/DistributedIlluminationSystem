#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <stdint.h>

// Simulates the system's response to a step change in duty cycle at the LED.
class Simulator {
public:
    Simulator();

    // Initialize the simulator
    void initialize(uint64_t initialTime, double initialVoltage, double initialDuty) volatile;

    // Get current predicted value
    double getLuminosity(uint64_t time) volatile;
    // Step change
    void changeInput(uint64_t time, double duty, double currentVoltage) volatile;

private:

    // Convert voltage to lux and vice-versa
    double voltageToLux(double voltage) volatile;
    double luxToVoltage(double lux) volatile;

    // Interpolate the tau functions
    double timeConstant(double initialVoltage, double finalVoltage) volatile;

    // Precomputed values for voltage, as taus are indexed by voltage
    double finalVoltageAscending[10];
    double finalVoltageDescending[10];

    // State variables of the system.
    unsigned long currentInitialTime;
    double currentInitialVoltage, currentInitialDuty;
    double currentTimeConstant;
};

#endif // SIMULATOR_HPP