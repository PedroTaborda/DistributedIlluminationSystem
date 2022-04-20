#include "calibration.hpp"

#include <Arduino.h>

#include "globals.hpp"
#include "utilities.hpp"

void calibrateGain() {
    // Define the duty cycles of the calibration points
    float firstDuty = 0.0, secondDuty = 1.0;

    // Set the LED power and wait for the LDR to reach steady-state
    set_u(firstDuty);
    delay(3000);

    // Get the final voltage and measure the corresponding luminance
    float voltage = measureVoltage(50);
    float firstLux = LDRVoltageToLux(voltage);

    // Repeat for the second duty cycle
    set_u(secondDuty);
    delay(3000);

    // Get the final voltage and measure the corresponding luminance
    voltage = measureVoltage(50);
    float secondLux = LDRVoltageToLux(voltage);

    gain = (secondLux - firstLux) / (secondDuty - firstDuty);
    ambientIlluminance = firstLux - gain * firstDuty;
}

Calibrator::Calibrator() {}

bool Calibrator::waiting() { return isWaiting; }

void Calibrator::resetWait() {
    Serial.printf("Resetting wait\n");
    cancel_alarm(waitAlarmId);
    waitAlarmId = add_alarm_in_ms(
        WAIT_TIME_MS,
        [](long int, void* instance) -> long long int {
            ((Calibrator*)instance)->endWait();
            Serial.printf("Alarm callback");
            return 0;
        },
        this, true);
    // waitAlarmId = add_alarm_in_ms(WAIT_TIME_MS, clbck, this, true);
}

bool Calibrator::waitingIds() { return isWaitingId; }

void Calibrator::resetWaitId() {
    cancel_alarm(waitAlarmId);
    waitAlarmId = add_alarm_in_ms(
        ID_WAIT_TIME_MS,
        [](long int, void* instance) -> long long int {
            ((Calibrator*)instance)->endWaitId();
            return 0;
        },
        this, true);
}

float Calibrator::getGainId(signed char id) {
    if (id > highestId) return -1.f;
    return staticGains[id];
}

float Calibrator::getExternalLuminance() { return externalLuminance; }

signed char Calibrator::getHighestId() { return highestId; }

void Calibrator::setHighestId(signed char id) {
    if (id > highestId) highestId = id;
}

void Calibrator::calibrateGainId(signed char id) {
    // Wait for first measurement steady-state
    delay(STEADY_STATE_WAIT_MS);
    float firstDutyVoltage = measureVoltage(CALIBRATION_VOLTAGE_SAMPLES);
    // Give some slack after the measurements in order to allow for
    // some slight delays causing desynchronization.
    delay(SYNCRONIZATION_WAIT_MS);

    // Wait for second measurement steady-state
    delay(STEADY_STATE_WAIT_MS);
    float secondDutyVoltage = measureVoltage(CALIBRATION_VOLTAGE_SAMPLES);
    // Give some slack after the measurements in order to allow for
    // some slight delays causing desynchronization.
    delay(SYNCRONIZATION_WAIT_MS);

    // Determine the corresponding luminances
    float firstDutyLuminance = LDRVoltageToLux(firstDutyVoltage);
    float secondDutyLuminance = LDRVoltageToLux(secondDutyVoltage);

    // Determine the static gain
    staticGains[id] = (secondDutyLuminance - firstDutyLuminance) / (SECOND_DUTY_CALIBRATION - FIRST_DUTY_CALIBRATION);
}

void Calibrator::selfCalibrate(signed char selfId) {
    // Ask the controller to set the desired duty cycle
    // controller.setDutyCycle(FIRST_DUTY_CALIBRATION);
    analogWrite(LED_PIN, (int)(FIRST_DUTY_CALIBRATION * DAC_RANGE));
    // Wait for first measurement steady-state
    delay(STEADY_STATE_WAIT_MS);
    float firstDutyVoltage = measureVoltage(CALIBRATION_VOLTAGE_SAMPLES);

    // Give some slack after the measurements in order to allow for
    // some slight delays causing desynchronization.
    delay(SYNCRONIZATION_WAIT_MS);

    // Ask the controller to set the desired duty cycle
    // controller.setDutyCycle(SECOND_DUTY_CALIBRATION);
    analogWrite(LED_PIN, (int)(SECOND_DUTY_CALIBRATION * DAC_RANGE));
    // Wait for second measurement steady-state
    delay(STEADY_STATE_WAIT_MS);
    float secondDutyVoltage = measureVoltage(CALIBRATION_VOLTAGE_SAMPLES);

    // Determine the corresponding luminances
    float firstDutyLuminance = LDRVoltageToLux(firstDutyVoltage);
    float secondDutyLuminance = LDRVoltageToLux(secondDutyVoltage);

    // Determine the static gain
    staticGains[selfId] =
        (secondDutyLuminance - firstDutyLuminance) / (SECOND_DUTY_CALIBRATION - FIRST_DUTY_CALIBRATION);

    externalLuminance = secondDutyLuminance - staticGains[selfId] * SECOND_DUTY_CALIBRATION;

    // Give some slack after the measurements in order to allow for
    // some slight delays causing desynchronization.
    delay(SYNCRONIZATION_WAIT_MS);

    // Turn the light off, as to not disturb the other calibrations
    // controller.setDutyCycle(0.0f);
    analogWrite(LED_PIN, 0);
}

void Calibrator::endCalibration() {
    isWaiting = false;
    maestro = false;
}

bool Calibrator::isMaestro() { return maestro; }

void Calibrator::becomeMaestro() { maestro = true; }

void Calibrator::endWait() { isWaiting = false; }

void Calibrator::endWaitId() { isWaitingId = false; }

Calibrator calibrator;
