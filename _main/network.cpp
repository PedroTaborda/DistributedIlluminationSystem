#include "network.hpp"

#include <algorithm>

void Network::addNodeToNetwork(signed char id) {
    DEBUG_PRINT("Attempting to add node with id %d\n", id)
    if(numberDevices == MAX_DEVICES)
        return;

    for(uint8_t i = 0; i < numberDevices; i++)
        if(deviceList[i] == id)
            return;

    deviceList[numberDevices++] = (uint8_t)id;
    std::sort(deviceList, deviceList + numberDevices);
    DEBUG_PRINT("Added node with id %hhu. Network has %hhu nodes.\n", id, numberDevices)
}

void Network::resetNetwork() {
    numberDevices = 0;
}

bool Network::compareNetwork(uint8_t *network, uint8_t numberDevicesInput) {
    if(this->numberDevices != numberDevicesInput)
        return false;

    for(uint8_t i = 0; i < numberDevicesInput; i++) {
        for(uint8_t j = 0; j < numberDevicesInput; j++) {
            if(deviceList[i] == network[j])
                goto found_match;
        }
        return false;
found_match:
        ;
    }

    return true;
}

uint8_t* Network::getNetwork() {
    return deviceList;
}

uint8_t Network::getNumberNodesNetwork() {
    return numberDevices;
}

uint8_t Network::getIndexId(signed char id) {
    for(uint8_t i = 0; i < numberDevices; i++)
        if(deviceList[i] == (uint8_t)id) return i;

    return MAX_DEVICES;
}

Network network;