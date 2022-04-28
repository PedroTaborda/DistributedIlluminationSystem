#ifndef NETWORK_HPP
#define NETWORK_HPP

#include "globals.hpp"

inline constexpr int ALIVE_CHECK_TIME_MS = 5000;
inline constexpr int MAX_TIME_SINCE_ALIVE = 5;

class Network {
public:
    void addNodeToNetwork(signed char id);
    void resetNetwork();

    bool compareNetwork(uint8_t network[], uint8_t numberDevices);
    uint8_t* getNetwork();
    uint8_t getNumberNodesNetwork();
    uint8_t getIndexId(signed char id);

    void beginAliveCheck(repeating_timer* timerStruct);
    void stayAlive(signed char id);

    bool shouldEmmitAlive();
    void emmitAliveMessage();

private:

    static bool checkAlive(repeating_timer* timerStruct);

    uint8_t deviceList[MAX_DEVICES];
    uint8_t timeSinceAlive[MAX_DEVICES] = {0};
    bool alive[MAX_DEVICES] = {0};
    bool emmitAlive = true;
    uint8_t numberDevices = 0;

    alarm_id_t aliveCheckAlarm;
};

extern Network network;

#endif