#include "interface.hpp"

#include "calibration.hpp"
#include "globals.hpp"
#include "utilities.hpp"

#include <Arduino.h>

#include <cerrno>
#include <cstring>

const char *commandStrings[] = {"g", "d", "r", "o", "a", "w", "b", "s", "B", "c", "m"};
void (*commandFunctions[])(char *) = {getCommand, dutyCommand, referenceCommand,
                                      occupancyCommand, windupCommand,
                                      feedforwardCommand, feedbackCommand,
                                      streamCommand, bufferCommand, configCommand,
                                      simulatorCommand};

const int NUMBER_COMMANDS = sizeof(commandStrings) / sizeof(char *);

void parseSerial() {
    String serialBuffer = Serial.readStringUntil('\n');
    char *string = (char *)serialBuffer.c_str();

    const char *delimiter = " ";
    char *command;

    command = strsep(&string, delimiter);

    if(command == NULL) {
        Serial.println("Invalid command.");
        return;
    }

    for(unsigned int i = 0; i < NUMBER_COMMANDS; i++) {
        if(strcmp(command, commandStrings[i]) == 0) {
            commandFunctions[i](string);
            break;
        }
    }
}

void getCommand(char *arguments) {  
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *variableString = strsep(&arguments, delimiter);
    if(variableString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }

    // Try to convert the variable to a character
    if(strlen(variableString) > 1) {
        Serial.println("Invalid variable for Get command.");
        return;
    }
    char variableChar = variableString[0];

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    float externalIlluminance;

    switch(variableChar) {
    case 'd':
        Serial.print("d "); Serial.print(luminaireIndex); Serial.print(" "); 
        SAFE_M_ACCESS(
            Serial.println(dutyBuffer.getBegin(1), 4);
        )
        break;
    case 'r':
        Serial.print("r "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(controller.getReference(), 4);
        )
        break;
    case 'l':
        Serial.print("l "); Serial.print(luminaireIndex); Serial.print(" "); Serial.println(LDRVoltageToLux(measureVoltage(5)), 4);
        break;
    case 'o':
        Serial.print("o "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(controller.getOccupancy());
        )
        break;
    case 'a':
        Serial.print("a "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(controller.getAntiWindup() ? '1' : '0');
        )
        break;
    case 'w':
        Serial.print("w "); Serial.print(luminaireIndex); Serial.print(" ");
        Serial.println(controller.getFeedforward() ? '1' : '0');
        break;
    case 'b':
        Serial.print("b "); Serial.print(luminaireIndex); Serial.print(" ");
        Serial.println(controller.getFeedback() ? '1' : '0');
        break;
    case 'x':
        Serial.print("x "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            externalIlluminance = luminanceBuffer.getBegin(1) - gain * dutyBuffer.getBegin(1);
        )
        Serial.println(externalIlluminance, 4);
        break;
    case 'p':
        Serial.print("p "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(dutyBuffer.getBegin(1), 4);
        )
        break;
    case 't':
        Serial.print("t "); Serial.print(luminaireIndex); Serial.print(" ");
        Serial.println((float)micros() / 1e6);
        break;
    case 'e':
        Serial.print("e "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(energy, 4);
        )
        break;
    case 'v':
        Serial.print("v "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(visibilityAccumulator/sampleNumber, 4);
        )
        break;
    case 'V':
        Serial.print("V "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(visibilityAccumulator, 4);
        )
        break;
    case 'f':
        Serial.print("f "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(flickerAccumulator/sampleNumber, 4);
        )
        break;
    case 'F':
        Serial.print("F "); Serial.print(luminaireIndex); Serial.print(" ");
        SAFE_M_ACCESS(
            Serial.println(flickerAccumulator, 4);
        )
        break;
    default:
        Serial.println("No such variable.");
    }
}

void dutyCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the duty cycle to a real number
    char *endPointer;
    float dutyCycle = strtof(valueString, &endPointer);
    if(dutyCycle == 0.f && endPointer == valueString) {
        Serial.println("Invalid duty cycle for Duty command.");
        return;
    }

    if(dutyCycle < 0.f || dutyCycle > 1.0f)
        Serial.println("err");
    else {
        IntFloat aux;
        aux.from = dutyCycle;
        rp2040.fifo.push((uint32_t) 'd');
        rp2040.fifo.push(aux.to);
        Serial.println("ack");
    }
}

void referenceCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the duty cycle to a real number
    char *endPointer;
    float luxReference = strtof(valueString, &endPointer);
    if(luxReference == 0.f && endPointer == valueString) {
        Serial.println("Invalid duty cycle for Duty command.");
        return;
    }

    if(luxReference < 0.0f)
        Serial.println("err");
    else {
        IntFloat aux;
        aux.from = luxReference;
        rp2040.fifo.push((uint32_t) 'r');
        rp2040.fifo.push(aux.to);
        Serial.println("ack");
    }
}

void occupancyCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int occupancyState = (int) strtol(valueString, NULL, 10);
    if(occupancyState == 0 && errno == EINVAL) {
        Serial.println("Occupancy state is not recognized.");
        return;
    }

    if(occupancyState < 0 || occupancyState > 1)
        Serial.println("err");
    else {
        rp2040.fifo.push((uint32_t) 'o');
        rp2040.fifo.push((uint32_t) occupancyState);
        Serial.println("ack");
    }
}

void windupCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int windup = (int) strtol(valueString, NULL, 10);
    if(windup == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if(windup < 0 || windup > 1)
        Serial.println("err");
    else {
        rp2040.fifo.push((uint32_t) 'a');
        rp2040.fifo.push((uint32_t) windup);
        Serial.println("ack");
    }
}

void feedforwardCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int feedforward = (int) strtol(valueString, NULL, 10);
    if(feedforward == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if(feedforward < 0 || feedforward > 1)
        Serial.println("err");
    else {
        rp2040.fifo.push((uint32_t) 'w');
        rp2040.fifo.push((uint32_t) feedforward);
        Serial.println("ack");
    }
}

void feedbackCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int feedback = (int) strtol(valueString, NULL, 10);
    if(feedback == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if(feedback < 0 || feedback > 1)
        Serial.println("err");
    else {
        rp2040.fifo.push((uint32_t) 'b');
        rp2040.fifo.push((uint32_t) feedback);
        Serial.println("ack");
    }
}

void streamCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *variableString = strsep(&arguments, delimiter);
    if(variableString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }

    // Try to convert the variable to a character
    if(strlen(variableString) > 1) {
        Serial.println("Invalid variable for Get command.");
        return;
    }
    char variableChar = variableString[0];

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    switch(variableChar) {
    case 'd':
        streamer.toggleDutyStream();
        break;
    case 'l':
        streamer.toggleLumminanceStream();
        break;
    case 'j':
        streamer.toggleJitterStream();
        break;
    case 'i':
        streamer.toggleIntegralErrorStream();
        break;
    case 'e':
        streamer.toggleTrackingErrorStream();
        break;
    case 's':
        streamer.toggleSimulatorStream();
        break;
    case 'r':
        streamer.toggleReferenceStream();
        break;
    default:
        Serial.println("err");
        return;
    }
}

void bufferCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *variableString = strsep(&arguments, delimiter);
    if(variableString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }

    // Try to convert the variable to a character
    if(strlen(variableString) > 1) {
        Serial.println("Invalid variable for Get command.");
        return;
    }
    char variableChar = variableString[0];

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    switch(variableChar) {
    case 'd':
        streamDutyBuffer = true;
        break;
    case 'l':
        streamLuminanceBuffer = true;
        break;
    default:
        Serial.println("err");
        return;
    }

    Serial.println("ack");
}

void configCommand(char *arguments) {
    const char *delimiter = " ";
    char *subCommand = strsep(&arguments, delimiter);
    if(subCommand == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }
    
    // Try to convert the subcommand to a character
    if(strlen(subCommand) > 1) {
        Serial.println("Invalid config subcommand.");
        return;
    }
    char variableChar = subCommand[0];

    // Dump all configuration data:
    // Static gain
    // Taus
    // Controller gain
    if(variableChar == 'd') {
        Serial.print("Static gain: "); Serial.println(gain);
        Serial.println("Ascending tau.");
        for(unsigned int i = 0; i < 10; i++) {
            Serial.print("Final lux: "); Serial.print(luxAscending[i]); Serial.print(" lux");
            Serial.print("\tTau: "); Serial.print(tauAscending[i]); Serial.println(" s");
        }
        Serial.print('\n');
        Serial.println("Descending tau.");
        for(unsigned int i = 0; i < 10; i++) {
            Serial.print("Final lux: "); Serial.print(luxDescending[i]); Serial.print(" lux");
            Serial.print("\tTau: "); Serial.print(tauDescending[i]); Serial.println(" s");
        }

        Serial.print("Proportional gain: "); 
        Serial.println(controller.getProportionalGain());

        Serial.print("Integral gain: "); 
        Serial.println(controller.getIntegralGain());
    } else if(variableChar == 'p') {
        char *valueString = strsep(&arguments, delimiter);
        if(valueString == NULL) {
            Serial.println("Invalid Duty command.");
            return;
        }
        char *endPointer;
        float proportionalGain = strtof(valueString, &endPointer);
        if(proportionalGain == 0.f && endPointer == valueString) {
            Serial.println("err");
            return;
        }

        if(proportionalGain < 0.f) {
            Serial.println("err");
            return;
        }
        SAFE_M_ACCESS(
            controller.setProportionalGain(proportionalGain);
        )
    } else if(variableChar == 'i') {
        char *valueString = strsep(&arguments, delimiter);
        if(valueString == NULL) {
            Serial.println("Invalid Duty command.");
            return;
        }
        char *endPointer;
        float integralGain = strtof(valueString, &endPointer);
        if(integralGain == 0.f && endPointer == valueString) {
            Serial.println("err");
            return;
        }

        if(integralGain < 0.f) {
            Serial.println("err");
            return;
        }
        SAFE_M_ACCESS(
            if(fequal(integralGain, 0.f) && controller.getAntiWindup() == 1)
                    controller.toggleAntiWindup();
            controller.setIntegralGain(integralGain);
        )
    } else if(variableChar == 'c') {
        if(!rp2040.fifo.push_nb((uint32_t) 'c'))
            Serial.println("err");
        else {
            rp2040.fifo.pop();
            Serial.println("ack");
        }
    }
}

void simulatorCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if(luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if(valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int) strtol(luminaireString, NULL, 10);
    if(luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int simulator = (int) strtol(valueString, NULL, 10);
    if(simulator == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if(simulator < 0 || simulator > 1)
        Serial.println("err");
    else {
        rp2040.fifo.push((uint32_t) 'm');
        rp2040.fifo.push((uint32_t) simulator);
        Serial.println("ack");
    }
}

VariableStream::VariableStream() :
streamLuminance(false),
streamDuty(false),
streamJitter(false),
streamIntegralError(false),
streamTrackingError(false)
{

}

void VariableStream::toggleLumminanceStream() {
    if(streamLuminance)
        Serial.println("ack");

    streamLuminance = !streamLuminance;
}

void VariableStream::toggleDutyStream() {
    if(streamDuty)
        Serial.println("ack");

    streamDuty = !streamDuty;
}

void VariableStream::toggleJitterStream() {
    if(streamJitter)
        Serial.println("ack");

    streamJitter = !streamJitter;
}

void VariableStream::toggleIntegralErrorStream() {
    if(streamIntegralError)
        Serial.println("ack");

    streamIntegralError = !streamIntegralError;
}

void VariableStream::toggleTrackingErrorStream() {
    if(streamTrackingError)
        Serial.println("ack");

    streamTrackingError = !streamTrackingError;
}

void VariableStream::toggleSimulatorStream() {
    if(streamSimulator)
        Serial.println("ack");

    streamSimulator = !streamSimulator;
}

void VariableStream::toggleReferenceStream() {
    if(streamReference)
        Serial.println("ack");

    streamReference = !streamReference;
}

void VariableStream::setNewValues() {
    newValues = true;
}

void VariableStream::streamVariables() {
    if(!newValues)
        return;

    IntFloat intfloat;
    if(streamLuminance) {        
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 'l');
        intfloat.from = luminanceBuffer.getBegin(1);
        rp2040.fifo.push(intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    if(streamDuty) {
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 'd');
        intfloat.from = dutyBuffer.getBegin(1);
        rp2040.fifo.push(intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    if(streamJitter) {
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 'j');
        intfloat.from = (float) controller.jitter;
        rp2040.fifo.push((uint32_t) intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    if(streamIntegralError) {
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 'i');
        intfloat.from = controller.integralError;
        rp2040.fifo.push(intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    if(streamTrackingError) {
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 'e');
        intfloat.from = controller.trackingError;
        rp2040.fifo.push(intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    if(streamSimulator) {
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 's');
        intfloat.from = controller.simulatorValue;
        rp2040.fifo.push(intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    if(streamReference) {
        rp2040.fifo.push((uint32_t) 's');
        rp2040.fifo.push((uint32_t) 'r');
        intfloat.from = controller.reference[controller.occupancy];
        rp2040.fifo.push(intfloat.to);
        rp2040.fifo.push((uint32_t) controller.lastTimestamp);
    }

    newValues = false;
}