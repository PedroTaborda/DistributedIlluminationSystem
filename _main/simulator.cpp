#include "simulator.hpp"

#include "globals.hpp"
#include "math_utils.hpp"
#include "utilities.hpp"

#include <Arduino.h>

#include <cmath>

Simulator::Simulator()
{
    range(tauN, finalVoltageAscending , 0.1, 1.0);
    range(tauN, finalVoltageDescending, 0.9, 0.0);

    for(int i = 0; i < tauN; i++)
    {
        tauAscending[i]  = l2v(d2l(tauAscending[i]));
        tauDescending[i] = l2v(d2l(tauDescending[i]));
    }
}

void Simulator::initialize(uint64_t initialTime, double initialVoltage, double finalVoltage) volatile
{
    currentInitialTime = initialTime;
    currentInitialVoltage = initialVoltage;
    currentFinalVoltage = finalVoltage;
}

double Simulator::getLuminosity(uint64_t time) volatile{
    // Take the current time as given by the microcontroller and determine the difference to the
    // the last switch instant. Then, convert it to seconds.
    double timeSinceSwitch = (double)(time - currentInitialTime);
    timeSinceSwitch /= 1e6;

    // Predict the current voltage using a first order model.
    double predictedVoltage = currentFinalVoltage - (currentFinalVoltage - currentInitialVoltage)
                             * exp(-timeSinceSwitch/currentTimeConstant);

    // Use that predicted voltage to predict current luminosity measurement.
    double predictedLux = voltageToLux(predictedVoltage);

    return predictedLux;
}

void Simulator::changeInput(uint64_t time, double goalVoltage, double currentVoltage) volatile{
    currentInitialTime = time;
    currentInitialVoltage = currentVoltage;
    currentTimeConstant = timeConstant(currentVoltage, goalVoltage);
    currentFinalVoltage = goalVoltage;
}

double Simulator::voltageToLux(double voltage) volatile{
    double predictedResistance = 10000 * (3.3 - voltage) / voltage;

    double predictedLux = pow(10, log10(225000 / predictedResistance) / gammaFactor + 1);

    return predictedLux;
}

double Simulator::luxToVoltage(double lux) volatile{
    double predictedResistance = 225000 / pow(10, gammaFactor * log10(lux / 10));

    double predictedVoltage = 3.3 * 10000 / (10000 + predictedResistance);

    return predictedVoltage;
}

double Simulator::timeConstant(double initialVoltage, double finalVoltage) volatile{
    if (initialVoltage < finalVoltage)
        return interpolate(tauN, (double *)finalVoltageAscending, tauAscending, finalVoltage);
    else
        return interpolate(tauN, (double *)finalVoltageDescending, tauDescending, finalVoltage);
}
