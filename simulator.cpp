#include "simulator.hpp"

#include "globals.hpp"

#include <Arduino.h>

#include <cmath>

Simulator::Simulator()
{
}

void Simulator::initialize(unsigned long initialTime, double initialVoltage, double initialDuty) volatile
{
    currentInitialTime = initialTime;
    currentInitialVoltage = initialVoltage;
    currentInitialDuty = initialDuty;
}

double Simulator::getLuminosity(unsigned long time) volatile{
    // Take the current time as given by the microcontroller and determine the difference to the
    // the last switch instant. Then, convert it to seconds.
    double timeSinceSwitch = (double)(time - currentInitialTime);
    timeSinceSwitch /= 1e6;

    // Determine the illumination associated to the current duty cycle, using the system's static
    // gain.
    double predictedFinalLux = gain * currentInitialDuty + ambientIlluminance;

    // Convert the predicted illumination into a predicted steady state voltage.
    double predictedFinalVoltage = luxToVoltage(predictedFinalLux);
    
    // Predict the current voltage using a first order model.
    double predictedVoltage = predictedFinalVoltage - (predictedFinalVoltage - currentInitialVoltage)
                             * exp(-timeSinceSwitch/currentTimeConstant);

    // Use that predicted voltage to predict current luminosity measurement.
    double predictedLux = voltageToLux(predictedVoltage);

    return predictedLux;
}

void Simulator::changeInput(unsigned long time, double duty, double currentVoltage) volatile{
    currentInitialTime = time;
    currentInitialVoltage = currentVoltage;
    currentTimeConstant = timeConstant(currentInitialDuty, duty);
    currentInitialDuty = duty;
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

double Simulator::timeConstant(double oldDuty, double newDuty) volatile{
    double oldLux = gain * oldDuty + ambientIlluminance;
    double newLux = gain * newDuty + ambientIlluminance;

    if(oldLux < newLux)
        return ascendingTimeConstant(newLux);
    else
        return descendingTimeConstant(newLux);
}

double Simulator::ascendingTimeConstant(double lux) volatile{
    int index = 9;
    if(lux <= luxAscending[0])
        return tauAscending[0];
    else if(lux >= luxAscending[9])
        return tauAscending[9];
    else
        for(;lux < luxAscending[index]; index--);

    double remainder = (lux - luxAscending[index]) / (luxAscending[index + 1] - luxAscending[index]);
    double tau = tauAscending[index] * (1 - remainder) + tauAscending[index + 1] * remainder;

    return tau;
}

double Simulator::descendingTimeConstant(double lux) volatile{
    int index = 9;
    if(lux <= luxDescending[0])
        return tauDescending[0];
    else if(lux >= luxDescending[9])
        return tauDescending[9];
    else
        for(;lux < luxDescending[index]; index--);

    double remainder = (lux - luxDescending[index]) / (luxDescending[index + 1] - luxDescending[index]);
    double tau = tauDescending[index] * (1 - remainder) + tauDescending[index + 1] * remainder;

    return tau;
}