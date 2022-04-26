#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include "globals.hpp"
#include "controller.hpp"

#include <pico/stdlib.h>

void calibrateGain();

void loadParamsStartup();

char *calibrateCommand(const char *args);

char *saveCalibrationCommand(const char *args);

char *printCalibratedCommand();

char *calibrateAutoCommand();

inline constexpr unsigned long WAIT_TIME_MS = 6000;
inline constexpr unsigned long ID_WAIT_TIME_MS = 3000;
inline constexpr unsigned long STEADY_STATE_WAIT_MS = 100;
inline constexpr unsigned long SYNCRONIZATION_WAIT_MS = 500;

inline constexpr float FIRST_DUTY_CALIBRATION = 0.0f;
inline constexpr float SECOND_DUTY_CALIBRATION = 1.0f;
inline constexpr unsigned int CALIBRATION_VOLTAGE_SAMPLES = 20;

class Calibrator{
public:
    Calibrator();

    bool waiting();
    void resetWait();

    double getGainId(signed char id);
    double *getGains();
    float getExternalLuminance();

    void calibrateGainId(signed char id);
    void selfCalibrate(signed char selfId);
    void endCalibration();

    bool isMaestro();
    void becomeMaestro();

private:

    void endWait();

    signed char highestId;
    double staticGains[MAX_DEVICES];
    double externalLuminance;

    volatile bool isWaiting = true;
    bool maestro = false;
    alarm_id_t waitAlarmId;
};

extern Calibrator calibrator;
#endif //CALIBRATION_HPP
