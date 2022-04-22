#include "utilities.hpp"

#include <math.h>

#include <algorithm>

#include "globals.hpp"

float LDRVoltageToLux(float voltage)
{
    float R_LDR = v2r(voltage);
    float LUX = r2l(R_LDR);
    return LUX;
}

float d2l(float d)
{
    return d * (gain ? gain : 10.0) + ambientIlluminance;
}

float l2d(float l)
{
    return (l - ambientIlluminance) / (gain ? gain : 10.0);
}

float v2l(float v)
{ // same as LDRVoltageToLux, but with "standard" naming
    return r2l(v2r(v));
}

float r2l(float r)
{
    // Determine the LUX measured by the LDR using the relationship
    // between its resistance and the LUX.
    return pow(10, log10(225000 / r) / (gammaFactor?gammaFactor:0.8) + 1);
}

float v2r(float v)
{
    // Determine the LDR's resistance from the voltage divider.
    float R = 10000.0;
    float Vcc = 3.3;
    float Rldr = (R * Vcc / v) - R;
    return Rldr;
}

float l2r(float l)
{
    float b = log10(225000) - (-gammaFactor);
    float logRldr = (-gammaFactor) * log10(l) + b;
    return pow(10, logRldr);
}

float l2v(float l)
{
    float R = 10000.0;
    float Vcc = 3.3;
    float Rldr = l2r(l);
    return Vcc * R / (Rldr + R);
}

float measureVoltage(int numberSamples) {
    // Sample the voltage divider circuit numberSamples times
    if (numberSamples > MAX_VOLTAGE_SAMPLES) return -1.0f;
    float samples[MAX_VOLTAGE_SAMPLES];
    for (int i = 0; i < numberSamples; i++) {
        int adcRead = analogRead(A0);
        samples[i] = (float)adcRead / (4096 - 1) * 3.3;
    }

    // Determine the median of the samples and return that
    // as the measured value.
    std::sort(samples, samples + numberSamples);

    return samples[numberSamples / 2];
}

void set_u(float duty) { analogWrite(LED_PIN, (int)(duty * DAC_RANGE)); }

bool fequal(float left, float right) {
    float relativeDifference = (left - right) / fmax(fabs(left), fabs(right));
    return fabs(relativeDifference) <= FLOAT_RELATIVE_TOLERANCE;
}