#ifndef NETWORK_HPP
#define NETWORK_HPP

#include "globals.hpp"

class Network {
public:
    void addNodeToNetwork(signed char id);
    void resetNetwork();

    bool compareNetwork(uint8_t network[], uint8_t numberDevices);
    uint8_t* getNetwork();
    uint8_t getNumberNodesNetwork();
    uint8_t getIndexId(signed char id);

private:

    uint8_t deviceList[MAX_DEVICES];
    uint8_t numberDevices = 0;
};

extern Network network;

#endif