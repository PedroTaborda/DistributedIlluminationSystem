#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

// Simulates the system's response to a step change in duty cycle at the LED.
class Simulator {
public:
    Simulator();

    // Initialize the simulator
    void initialize(unsigned long initialTime, double initialVoltage, double initialDuty) volatile;

    // Get current predicted value
    double getLuminosity(unsigned long time) volatile;
    // Step change
    void changeInput(unsigned long time, double duty, double currentVoltage) volatile;

private:

    // Convert voltage to lux and vice-versa
    double voltageToLux(double voltage) volatile;
    double luxToVoltage(double lux) volatile;

    // Interpolate the tau functions
    double timeConstant(double oldDuty, double newDuty) volatile;
    double ascendingTimeConstant(double duty) volatile;
    double descendingTimeConstant(double duty) volatile;

    // State variables of the system.
    unsigned long currentInitialTime;
    double currentInitialVoltage, currentInitialDuty;
    double currentTimeConstant;
};

#endif // SIMULATOR_HPP