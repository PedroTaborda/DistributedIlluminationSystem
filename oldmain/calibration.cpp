#include "calibration.hpp"

#include <Arduino.h>

#include "globals.hpp"
#include "utilities.hpp"
#include "math_utils.hpp"



void calibrateGain() {
    // Define the duty cycles of the calibration points
    float firstDuty = 0.1, secondDuty = 1.0;

    // Set the LED power and wait for the LDR to reach steady-state
    set_u(firstDuty);
    delay(STEADY_STATE_WAIT_MS);

    // Get the final voltage and measure the corresponding luminance
    float voltage = measureVoltage(50);
    float firstLux = LDRVoltageToLux(voltage);

    // Repeat for the second duty cycle
    set_u(secondDuty);
    delay(STEADY_STATE_WAIT_MS);

    // Get the final voltage and measure the corresponding luminance
    voltage = measureVoltage(50);
    float secondLux = LDRVoltageToLux(voltage);

    gain = (secondLux - firstLux) / (secondDuty - firstDuty);
    ambientIlluminance = firstLux - gain * firstDuty;
}

void calibrateGamma(int nPoints, int delayTimeMs, float startu, float endu){
    double logR[nPoints];
    double logL[nPoints];

    double ustep = (endu - startu) / (nPoints - 1);
    double r = 0.0;
    double l = 0.0;

    for (int i = 0; i < nPoints; i++){
        set_u(startu + ustep * i);
        delay(delayTimeMs);
        r = v2r(measureVoltage(11));
        l = r2l(r);
        logR[i] = log10(r);
        logL[i] = log10(l);
    }
    gammaFactor = -linearRegression(nPoints, logL, logR).slope;
}

void calibrateGamma(){
    calibrateGamma(50, 2000, 0.1, 1.0);
}

#define SAMPLE_TIME_MS 10
#define SAMPLE_TIME_US 1000 * SAMPLE_TIME_MS
#define TRIAL_END_THRESHOLD 0.8
#define MAX_TAU_TRIAL_DURATION 2000

double estimateTauTrial(float V0, float Vf, float maxTrialDurationMS)
{
    unsigned int N = maxTrialDurationMS / SAMPLE_TIME_MS;
    unsigned int n = N;

    // these time representations use the 64-bit microsecond timer, valid
    // for millions of years
    absolute_time_t t0_us;      
    absolute_time_t t_cur_us;

    double y[N] = {0.0};
    double x[N] = {0.0};

    float V = 0.0;

    set_u(l2d(v2l(Vf)));

    for (unsigned int i = 0; i < N; i++)
    {
        V = measureVoltage(11);
        t_cur_us = get_absolute_time();
        if (i == 0)
            t0_us = t_cur_us;
        if ((V - V0) / (Vf - V0) > TRIAL_END_THRESHOLD)
        {
            n = i;
            break;
        }
        y[i] = ((double)absolute_time_diff_us(t0_us, t_cur_us)) / 1e6;
        x[i] = log((Vf - V) / (Vf - V0));
        busy_wait_until(delayed_by_us(t0_us, SAMPLE_TIME_US*(i + 1)));
    }

    return linearRegression(n, x, y).slope;
}

void calibrateTau(unsigned int nTrials, unsigned int tau_steps, float *tau_up, float *tau_down)
{
    unsigned int N = nTrials;
    double V0 = 0.0;
    double Vf = 0.0;

    double buf_tau[N] = {0.0};

    // Trials for ascent values
    for (unsigned int i = 0; i < tau_steps; i++)
    {
        V0 = measureVoltage(11);
        Vf = l2v(d2l(0.1 + 0.9 * ((double)i) / ((double)tau_steps - 1.0)));
        for (unsigned int j = 0; j < N; j++)
        {
            set_u(0.0);
            delay(STEADY_STATE_WAIT_MS);
            DEBUG_PRINT("V0: %.3f, Vf: %.3f\n", V0, Vf)
            buf_tau[j] = estimateTauTrial(V0, Vf, MAX_TAU_TRIAL_DURATION);
            DEBUG_PRINT("tau_asc[%d][%d] %.3f\n", i, j, buf_tau[j])
        }

        tau_up[i] = mean(N, buf_tau);
        DEBUG_PRINT("Tau up [%d]: %.3f\n", i + 1, tau_up[i])
    }

    // Trials for descent values
    for (unsigned int i = 0; i < tau_steps; i++)
    {
        V0 = measureVoltage(11);
        Vf = l2v(d2l(0.9 - (0.9 * ((float)i) / ((float)tau_steps - 1))));
        for (unsigned int j = 0; j < N; j++)
        {
            set_u(1.0);
            delay(STEADY_STATE_WAIT_MS);
            Serial.printf("V0: %.3f, Vf: %.3f\n", V0, Vf);
            buf_tau[j] = estimateTauTrial(V0, Vf, MAX_TAU_TRIAL_DURATION);
            Serial.printf("tau_desc[%d][%d] %.3f\n", i, j, buf_tau[j]);
        }
        tau_down[i] = mean(N, buf_tau);
        Serial.printf("Tau down [%d]: %.3f\n", i + 1, tau_down[i]);
    }
}

Calibrator::Calibrator()
{
}

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
