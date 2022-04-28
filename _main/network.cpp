#include "calibration.hpp"
#include "consensus.hpp"
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

void Network::beginAliveCheck(repeating_timer* timerStruct) {
    add_repeating_timer_ms(-ALIVE_CHECK_TIME_MS, Network::checkAlive, (void *)this, timerStruct);
}

void Network::stayAlive(signed char id) {
    alive[getIndexId(id)] = true;
}

bool Network::shouldEmmitAlive() {
    return emmitAlive;
}

void Network::emmitAliveMessage() {
    int ret;
    SEND_MSG(0, RETRY_TIMEOUT_MS,
        Wire.write(MSG_TYPE_ALIVE);,
    ret)
    emmitAlive = false;
}

bool Network::checkAlive(repeating_timer* timerStruct) {
    Network *instance = (Network *)timerStruct->user_data;

    for(uint8_t i = 0; i < instance->numberDevices; i++) {
        if(myID == instance->deviceList[i]) continue;
        if(instance->alive[i]) {
            instance->timeSinceAlive[i] = 0;
            instance->alive[i] = false;
        }
        else
            instance->timeSinceAlive[i] += 1;

        if(instance->timeSinceAlive[i] >= MAX_TIME_SINCE_ALIVE) {
            memcpy(instance->deviceList + i, instance->deviceList + i + 1, instance->numberDevices - i - 1);
            instance->numberDevices -= 1;
            i -= 1;
            calibrator.removeNode(i);
            consensus.nNodes -= 1;
        }
    }

    instance->emmitAlive = true;

    return true;
}

Network network;