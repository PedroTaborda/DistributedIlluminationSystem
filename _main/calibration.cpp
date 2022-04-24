#include "calibration.hpp"

#include <Arduino.h>
#include <EEPROM.h>

#include "globals.hpp"
#include "utilities.hpp"
#include "math_utils.hpp"

#define SAMPLE_TIME_MS 1
#define SAMPLE_TIME_US 1000 * SAMPLE_TIME_MS
#define TRIAL_END_THRESHOLD 0.6
#define MAX_TAU_TRIAL_DURATION 2000

luminaireParams latestCalibration;
luminaireParams activeParams()
{
    luminaireParams params;
    params.gammaFactor = gammaFactor;
    for (int i = 0; i < tauN; i++)
    {
        params.tauAscending[i] = tauAscending[i];
        params.tauDescending[i] = tauDescending[i];
    }
    return params;
}

void calibrateGain()
{
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

double calibrateGamma(int nPoints, int delayTimeMs, float startu, float endu)
{
    double logR[nPoints];
    double logL[nPoints];

    float ustep = (endu - startu) / (nPoints - 1);
    float u = startu;
    double r = 0.0;

    float v = 0.0;

    DEBUG_PRINT("Calibrating gamma factor...\n");
    for (int i = 0; i < nPoints; i++)
    {
        u = startu + i * ustep;
        set_u(u);
        delay(delayTimeMs);
        v = measureVoltage(11);
        r = v2r(v);
        logR[i] = log10(r);
        logL[i] = log10(u);
        DEBUG_PRINT("%d: u: %f, v: %f, r: %f\n", i, u, v, r);
    }
    double newGamma = -linearRegression(nPoints, logL, logR).slope;
    DEBUG_PRINT("Gamma factor: %f\n", newGamma);
    return newGamma;
}

double estimateTauTrial(float V0, float Vf, float maxTrialDurationMS)
{
    unsigned int N = maxTrialDurationMS / SAMPLE_TIME_MS;
    unsigned int n = N;

    // these time representations use the 64-bit microsecond timer, valid
    // for millions of years
    absolute_time_t t0;
    absolute_time_t t;

    double y[N] = {0.0};
    double V[N] = {0.0};

    set_u(l2d(v2l(Vf)));

    for (unsigned int i = 0; i < N; i++)
    {
        V[i] = (double)measureVoltage(11);
        t = get_absolute_time();
        if (i == 0)
            t0 = t;
        y[i] = ((double)absolute_time_diff_us(t0, t)) / 1e6;
        DEBUG_PRINT("%d: V: %f, t: %f\n", i, V[i], y[i]);
        if ((V[i] - V0) / (Vf - V0) > TRIAL_END_THRESHOLD)
        {
            n = i+1;
            break;
        }
        busy_wait_until(delayed_by_us(t0, SAMPLE_TIME_US * (i + 1)));
    }

    if (n == N)
    {
        DEBUG_PRINT("Trial ended with max duration. Fixing Vf.\n");
        Vf = V[n-1] + 0.01*(Vf-V0)/abs(Vf-V0);
    }
    for (unsigned int i = 0; i < n; i++)
    {

        V[i] = log((Vf - V[i]) / (Vf - V0));
    }

    return -linearRegression(n, V, y).slope;
}

void calibrateTau(unsigned int nTrials, unsigned int tauSteps, float *tauUp, float *tauDown)
{
    double V0 = 0.0;
    double Vf = 0.0;

    double tauBuffer[nTrials] = {0.0};


    DEBUG_PRINT("Calibrating tau...\n");
    // Trials for ascent values
    for (unsigned int i = 0; i < tauSteps; i++)
    {
        for (unsigned int j = 0; j < nTrials; j++)
        {
            do{
                set_u(0.0);
                delay(STEADY_STATE_WAIT_MS);
                V0 = measureVoltage(11);
                Vf = l2v(d2l(0.1 + 0.9 * ((double)i) / ((double)tauSteps - 1.0)));
                DEBUG_PRINT("V0: %.3f, Vf: %.3f\n", V0, Vf)
                tauBuffer[j] = estimateTauTrial(V0, Vf, MAX_TAU_TRIAL_DURATION);
                DEBUG_PRINT("tauAscending[%d][%d] %.3f\n", i, j, tauBuffer[j])
            }while(tauBuffer[j] < 0.0);
        }

        tauUp[i] = mean(nTrials, tauBuffer);
        DEBUG_PRINT("tauAscending[%d]: %.3f\n", i, tauUp[i])
    }

    // Trials for descent values
    for (unsigned int i = 0; i < tauSteps; i++)
    {
        for (unsigned int j = 0; j < nTrials; j++)
        {
            do{
                set_u(1.0);
                delay(STEADY_STATE_WAIT_MS);
                V0 = measureVoltage(11);
                Vf = l2v(d2l(0.9 - (0.9 * ((float)i) / ((float)tauSteps - 1))));
                DEBUG_PRINT("V0: %.3f, Vf: %.3f\n", V0, Vf)
                tauBuffer[j] = estimateTauTrial(V0, Vf, MAX_TAU_TRIAL_DURATION);
                DEBUG_PRINT("tauDescending[%d][%d] %.3f\n", i, j, tauBuffer[j])
            }while(tauBuffer[j] < 0.0);
        }
        tauDown[i] = mean(nTrials, tauBuffer);
        DEBUG_PRINT("tauDescending[%d]: %.3f\n", i, tauDown[i])
    }
}

luminaireParams calibrateSelf(bool doGammaCalibration, bool doTauCalibration)
{
    luminaireParams params = {0.0, {0.0}, {0.0}};

    if (doGammaCalibration){
        int nPoints = 50;
        int delayMs = 2000;
        float startu = 0.1;
        float endu = 1.0;
        params.gammaFactor = calibrateGamma(nPoints, delayMs, startu, endu);
    }
    else
        params.gammaFactor = gammaFactor;
    gammaFactor = gammaFactor? gammaFactor: params.gammaFactor;
    gammaFactor = gammaFactor? gammaFactor: 0.8;

    if(doGammaCalibration)
        calibrateGain();

    if (doTauCalibration){
        int nTrials = 5;
        calibrateTau(nTrials, tauN, params.tauAscending, params.tauDescending);
    }
    else
    {
        for (int i = 0; i < tauN; i++)
        {
            params.tauAscending[i] = tauAscending[i];
            params.tauDescending[i] = tauDescending[i];
        }
    }

    return params;
}

#ifdef DEBUG
void showParams(luminaireParams params)
{
    DEBUG_PRINT("gammaFactor: %.3f\n", params.gammaFactor)
    for (int i = 0; i < tauN; i++)
    {
        DEBUG_PRINT(" tauAscending[%d]: %.3f\n", i, params.tauAscending[i])
    }
    for (int i = 0; i < tauN; i++)
    {
        DEBUG_PRINT("tauDescending[%d]: %.3f\n", i, params.tauDescending[i])
    }
}
#endif

luminaireParams loadParamsEEPROM()
{
    luminaireParams params = {0.0, {0.0}, {0.0}};
    unsigned int paramsSize = sizeof(luminaireParams);
    unsigned int storedSize = 0;

    EEPROM.begin(4096);
    EEPROM.get(0, storedSize);

    if (storedSize == paramsSize)
    {
        EEPROM.get(sizeof(unsigned int), params);
    }
    else
    {
        DEBUG_PRINT("EEPROM size mismatch: got %dB, expected %dB\n", storedSize, paramsSize)
    }

    return params;
}

void saveParamsEEPROM(luminaireParams params)
{
    unsigned int paramsSize = sizeof(luminaireParams);
    EEPROM.begin(4096);
    EEPROM.put(0, paramsSize);
    EEPROM.put(sizeof(unsigned int), params);
    EEPROM.commit();
}

char *calibrateCommand(const char *args)
{
    int doGammaCalibration = 0;
    int doTauCalibration = 0;
    static char ret_str[] = "Calibration successful";
    int ret = sscanf(args, "%d %d", &doGammaCalibration, &doTauCalibration);
    if (ret != 2)
    {
        DEBUG_PRINT("Invalid arguments for calibrateCommand. Expected 2, got %d\n", ret)
        DEBUG_PRINT("Argument string received: %s\n", args)
        return NULL;
    }
    DEBUG_PRINT("Calibrating self with doGammaCalibration: %d, doTauCalibration: %d\n", doGammaCalibration?1:0, doTauCalibration?1:0)
    latestCalibration = calibrateSelf((bool)doGammaCalibration, (bool)doTauCalibration);
    return ret_str;
}

char *saveCalibrationCommand(const char *args)
{
    int saveGammaCalibration = 0;
    int saveTauCalibration = 0;
    static char retStr[] = "Successfully saved calibration";
    static char retStrNone[] = "No parameters to save";
    int ret = sscanf(args, "%d %d", &saveGammaCalibration, &saveTauCalibration);
    if (ret != 2)
    {
        DEBUG_PRINT("Invalid arguments for saveCalibrationCommand. Expected 2, got %d\n", ret)
        return NULL;
    }
    luminaireParams paramsToSave;
    if (saveGammaCalibration && saveTauCalibration){
        DEBUG_PRINT("Saving calibration with gamma and tau\n")
        paramsToSave = latestCalibration;
    }
    if (saveGammaCalibration && !saveTauCalibration){
        DEBUG_PRINT("Saving calibration with gamma\n")
        paramsToSave = activeParams();
        paramsToSave.gammaFactor = latestCalibration.gammaFactor;
    }
    if (!saveGammaCalibration && saveTauCalibration){
        DEBUG_PRINT("Saving calibration with tau\n")
        paramsToSave = latestCalibration;
        paramsToSave.gammaFactor = gammaFactor;
    }
    if (!saveGammaCalibration && !saveTauCalibration){
        DEBUG_PRINT("No parameters to save\n")
        return retStrNone;
    }
    saveParamsEEPROM(paramsToSave);
    return retStr;
}

char *printCalibratedCommand()
{
#ifdef DEBUG
    static char retStr[] = "Calibration shown successfully";
    DEBUG_PRINT("Current parameters:\n")
    showParams(activeParams());
    DEBUG_PRINT("Saved parameters:\n")
    luminaireParams params = loadParamsEEPROM();
    if (params.gammaFactor == 0.0)
        DEBUG_PRINT("No saved calibration\n")
    else
        showParams(params);
    DEBUG_PRINT("Latest calibration:\n")
    showParams(latestCalibration);
#else
    static char retStr[] = "Command available only in debug mode";
#endif
    return retStr;
}


void loadParamsStartup(){
    luminaireParams stored = loadParamsEEPROM();
    gammaFactor = stored.gammaFactor;
    for (int i = 0; i < tauN; i++)
    {
        tauAscending[i] = stored.tauAscending[i];
        tauDescending[i] = stored.tauDescending[i];
    }
    latestCalibration = stored;
}

char *calibrateAutoCommand(){
    static char retStr[] = "Calibration successful";
    DEBUG_PRINT("Calibrating self with automatic calibration\n")
    latestCalibration = calibrateSelf(true, true); // calibrate gamma and tau
    saveParamsEEPROM(latestCalibration); // save calibration to EEPROM
    loadParamsStartup(); // make these the active parameters
    return retStr;
}

Calibrator::Calibrator() {
}

bool Calibrator::waiting() {
    return isWaiting;
}

void Calibrator::resetWait() {
    DEBUG_PRINT("Resetting wait\n")
    cancel_alarm(waitAlarmId);
    waitAlarmId = add_alarm_in_ms(WAIT_TIME_MS, [](long int, void* instance) 
                                  -> long long int {((Calibrator*) instance)->endWait(); return 0;}, this, true);
}

bool Calibrator::waitingIds() {
    return isWaitingId;
}

void Calibrator::resetWaitId() {
    cancel_alarm(waitAlarmId);
    waitAlarmId = add_alarm_in_ms(ID_WAIT_TIME_MS, [](long int, void* instance) 
                                  -> long long int {((Calibrator*) instance)->endWaitId(); return 0;}, this, true);
}

float Calibrator::getGainId(signed char id) {
    if(id > highestId)
        return -1.f;
    return staticGains[id];
}

float Calibrator::getExternalLuminance() {
    return externalLuminance;
}

signed char Calibrator::getHighestId() {
    return highestId;
}

void Calibrator::setHighestId(signed char id) {
    if(id > highestId)
        highestId = id;
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
    staticGains[id] = (secondDutyLuminance - firstDutyLuminance) / \
                      (SECOND_DUTY_CALIBRATION - FIRST_DUTY_CALIBRATION);
}

void Calibrator::selfCalibrate(signed char selfId) {
    // Ask the controller to set the desired duty cycle
    //controller.setDutyCycle(FIRST_DUTY_CALIBRATION);
    analogWrite(LED_PIN, (int) (FIRST_DUTY_CALIBRATION * DAC_RANGE));
    // Wait for first measurement steady-state
    delay(STEADY_STATE_WAIT_MS);
    float firstDutyVoltage = measureVoltage(CALIBRATION_VOLTAGE_SAMPLES);

    // Give some slack after the measurements in order to allow for
    // some slight delays causing desynchronization.
    delay(SYNCRONIZATION_WAIT_MS);

    // Ask the controller to set the desired duty cycle
    //controller.setDutyCycle(SECOND_DUTY_CALIBRATION);
    analogWrite(LED_PIN, (int) (SECOND_DUTY_CALIBRATION * DAC_RANGE));
    // Wait for second measurement steady-state
    delay(STEADY_STATE_WAIT_MS);
    float secondDutyVoltage = measureVoltage(CALIBRATION_VOLTAGE_SAMPLES);

    // Determine the corresponding luminances
    float firstDutyLuminance = LDRVoltageToLux(firstDutyVoltage);
    float secondDutyLuminance = LDRVoltageToLux(secondDutyVoltage);

    // Determine the static gain
    staticGains[selfId] = (secondDutyLuminance - firstDutyLuminance) / \
                      (SECOND_DUTY_CALIBRATION - FIRST_DUTY_CALIBRATION);

    externalLuminance = secondDutyLuminance - staticGains[selfId] * SECOND_DUTY_CALIBRATION;

    // Give some slack after the measurements in order to allow for
    // some slight delays causing desynchronization.
    delay(SYNCRONIZATION_WAIT_MS);

    // Turn the light off, as to not disturb the other calibrations
    //controller.setDutyCycle(0.0f);
    analogWrite(LED_PIN, 0);
}

void Calibrator::endCalibration() {
    isWaiting = false;
    maestro = false;
}

bool Calibrator::isMaestro() {
    return maestro;
}

void Calibrator::becomeMaestro() {
    maestro = true;
}

void Calibrator::endWait() {
    isWaiting = false;
}

void Calibrator::endWaitId() {
    isWaitingId = false;
}

Calibrator calibrator;
