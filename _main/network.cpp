#include "network.hpp"

void Network::addNodeToNetwork(signed char id) {
    if(numberDevices == MAX_DEVICES)
        return;

/*    for(int i = 0; i < numberDevices; i++)
        if(deviceList[i] == id)
            return;*/

    deviceList[numberDevices++] = id;
}

void Network::resetNetwork() {
    numberDevices = 0;
}

bool Network::compareNetwork(uint8_t network[], uint8_t numberDevices) {
    if(this->numberDevices != numberDevices)
        return false;

    for(uint8_t i = 0; i < numberDevices; i++) {
        for(uint8_t j = 0; j < numberDevices; j++) {
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
        if(deviceList[i] == id) return i;

    return MAX_DEVICES;
}

Network network;