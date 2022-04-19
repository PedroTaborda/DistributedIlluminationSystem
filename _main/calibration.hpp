#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <pico/stdlib.h>

//void calibrateGain();

inline constexpr unsigned long WAIT_TIME_MS = 1000;
inline constexpr unsigned long ID_WAIT_TIME_MS = 100;
inline constexpr unsigned long STEADY_STATE_WAIT_MS = 3000;
inline constexpr unsigned long SYNCRONIZATION_WAIT_MS = 500;
//Place this somewhere else perhaps
inline constexpr unsigned int MAX_DEVICES = 16;

inline constexpr float FIRST_DUTY_CALIBRATION = 0.0f;
inline constexpr float SECOND_DUTY_CALIBRATION = 1.0f;
inline constexpr unsigned int CALIBRATION_VOLTAGE_SAMPLES = 20;

class Calibrator{
public:
    
    Calibrator();

    bool waiting();
    void resetWait();
    
    bool waitingIds();
    void resetWaitId();

    float getGainId(signed char id);
    float getExternalLuminance();

    signed char getHighestId();
    void setHighestId(signed char id);

    void calibrateGainId(signed char id);
    void selfCalibrate(signed char selfId);
    void endCalibration();

    bool isMaestro();
    void becomeMaestro();

private:

    void endWait();
    void endWaitId();

    signed char highestId;
    float staticGains[MAX_DEVICES];
    float externalLuminance;

    bool isWaiting = true;
    bool isWaitingId = true;
    bool maestro = false;
    alarm_id_t waitAlarmId;
};

extern Calibrator calibrator;
#endif //CALIBRATION_HPP
