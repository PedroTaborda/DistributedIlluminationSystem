#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include <Arduino.h>

#include <cerrno>
#include <cstring>

#include "calibration.hpp"
#include "globals.hpp"
#include "parser.hpp"
#include "utilities.hpp"

void parseSerial();

// Functions for the various interface commands
void getCommand(char *arguments);
void dutyCommand(char *arguments);
void referenceCommand(char *arguments);
void occupancyCommand(char *arguments);
void windupCommand(char *arguments);
void feedforwardCommand(char *arguments);
void feedbackCommand(char *arguments);
void streamCommand(char *arguments);
void bufferCommand(char *arguments);
void configCommand(char *arguments);
void simulatorCommand(char *arguments);
void streamVariables();

// // Helper class for the stream command
// class VariableStream {
//    public:
//     VariableStream();

//     void toggleDutyStream();
//     void toggleLumminanceStream();
//     void toggleJitterStream();
//     void toggleIntegralErrorStream();
//     void toggleTrackingErrorStream();
//     void toggleSimulatorStream();
//     void toggleReferenceStream();

//     void setNewValues();
//     void streamVariables();

//    private:
//     bool streamLuminance;
//     bool streamDuty;
//     bool streamJitter;
//     bool streamIntegralError;
//     bool streamTrackingError;
//     bool streamSimulator;
//     bool streamReference;

//     bool newValues;
// };

#endif  // INTERFACE_HPP