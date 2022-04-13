#include "utilities.hpp"

#include "globals.hpp"

#include <math.h>

#include <algorithm>

float LDRVoltageToLux(float voltage) {
    float R = 10e3;             // Fixed resistance value
    float Vcc = 3.3;            // Fixed source voltage

    // Determine the LDR's resistance from the voltage divider.
    float R_LDR = R * (Vcc - voltage) / voltage;
    
    // Determine the LUX measured by the LDR using the relationship
    // between its resistance and the LUX.
    float LUX = pow(10, log10(225000/R_LDR)/gammaFactor + 1);

    return LUX;
}

float measureVoltage(int numberSamples) {
    // Sample the voltage divider circuit numberSamples times
    if(numberSamples > MAX_VOLTAGE_SAMPLES)
        return -1.0f;
    float samples[MAX_VOLTAGE_SAMPLES];
    for(int i = 0; i < numberSamples; i++) {
        int adcRead = analogRead(A0);
        samples[i] = (float) adcRead / (4096 - 1) * 3.3;
    }

    // Determine the median of the samples and return that
    // as the measured value.
    std::sort(samples, samples + numberSamples);

    return samples[numberSamples/2];
}

bool fequal(float left, float right) {
    float relativeDifference = (left - right) / fmax(fabs(left), fabs(right));
    return fabs(relativeDifference) <= FLOAT_RELATIVE_TOLERANCE;
}