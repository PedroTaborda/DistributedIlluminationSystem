#include "calibration.hpp"

#include <Arduino.h>

#include "globals.hpp"
#include "utilities.hpp"

void calibrateGain() {
    // Define the duty cycles of the calibration points
    float firstDuty = 0.0, secondDuty = 1.0;

    // Set the LED power and wait for the LDR to reach steady-state
    analogWrite(LED_PIN, (int) (firstDuty * DAC_RANGE));
    delay(3000);

    // Get the final voltage and measure the corresponding luminance
    float voltage = measureVoltage(50);
    float firstLux = LDRVoltageToLux(voltage);

    // Repeat for the second duty cycle
    analogWrite(LED_PIN, (int) (secondDuty * DAC_RANGE));
    delay(3000);

    // Get the final voltage and measure the corresponding luminance
    voltage = measureVoltage(50);
    float secondLux = LDRVoltageToLux(voltage);

    gain = (secondLux - firstLux) / (secondDuty - firstDuty);
    ambientIlluminance = firstLux - gain * firstDuty;
}