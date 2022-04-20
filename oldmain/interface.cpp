#include "interface.hpp"

const char *commandStrings[] = {"g", "d", "r", "o", "a", "w", "b", "s", "B", "c", "m"};
void (*commandFunctions[])(char *) = {getCommand,    dutyCommand,        referenceCommand, occupancyCommand,
                                      windupCommand, feedforwardCommand, feedbackCommand,  streamCommand,
                                      bufferCommand, configCommand,      simulatorCommand};

const int NUMBER_COMMANDS = sizeof(commandStrings) / sizeof(char *);

void parseSerial() {
    String serialBuffer = Serial.readStringUntil('\n');
    char *string = (char *)serialBuffer.c_str();

    const char *delimiter = " ";
    char *command;

    command = strsep(&string, delimiter);

    if (command == NULL) {
        Serial.println("Invalid command.");
        return;
    }

    for (unsigned int i = 0; i < NUMBER_COMMANDS; i++) {
        if (strcmp(command, commandStrings[i]) == 0) {
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
    if (variableString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }

    // Try to convert the variable to a character
    if (strlen(variableString) > 1) {
        Serial.println("Invalid variable for Get command.");
        return;
    }
    char variableChar = variableString[0];

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    float externalIlluminance;

    switch (variableChar) {
        case 'd':
            Serial.print("d ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getDutyCycle(), 4);
            break;
        case 'r':
            Serial.print("r ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getReference(), 4);
            break;
        case 'l':
            Serial.print("l ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(LDRVoltageToLux(measureVoltage(5)), 4);
            break;
        case 'o':
            Serial.print("o ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getOccupancy(), 4);
            break;
        case 'a':
            Serial.print("a ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getAntiWindup() ? '1' : '0');
            break;
        case 'w':
            Serial.print("w ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getFeedforward() ? '1' : '0');
            break;
        case 'b':
            Serial.print("b ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getFeedback() ? '1' : '0');
            break;
        case 'x':
            Serial.print("x ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            externalIlluminance = controller.getIlluminance() - gain * controller.getDutyCycle();
            Serial.println(externalIlluminance, 4);
            break;
        case 'p':
            Serial.print("p ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getDutyCycle(), 4);
            break;
        case 't':
            Serial.print("t ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println((double)micros() / 1e6);
            break;
        case 'e':
            Serial.print("e ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getEnergySpent(), 4);
            break;
        case 'v':
            Serial.print("v ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getVisibilityAccumulator() / controller.getSampleNumber(), 4);
            break;
        case 'V':
            Serial.print("V ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getVisibilityAccumulator(), 4);
            break;
        case 'f':
            Serial.print("f ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getFlickerAccumulator() / controller.getSampleNumber(), 4);
            break;
        case 'F':
            Serial.print("F ");
            Serial.print(luminaireIndex);
            Serial.print(" ");
            Serial.println(controller.getFlickerAccumulator(), 4);
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
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the duty cycle to a real number
    char *endPointer;
    float dutyCycle = strtof(valueString, &endPointer);
    if (dutyCycle == 0.f && endPointer == valueString) {
        Serial.println("Invalid duty cycle for Duty command.");
        return;
    }

    if (dutyCycle < 0.f || dutyCycle > 1.0f)
        Serial.println("err");
    else {
        controller.setDutyCycle(dutyCycle);
        Serial.println("ack");
    }
}

void referenceCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the duty cycle to a real number
    char *endPointer;
    float luxReference = strtof(valueString, &endPointer);
    if (luxReference == 0.f && endPointer == valueString) {
        Serial.println("Invalid duty cycle for Duty command.");
        return;
    }

    if (luxReference < 0.0f)
        Serial.println("err");
    else {
        controller.setReference(luxReference);
        Serial.println("ack");
    }
}

void occupancyCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int occupancyState = (int)strtol(valueString, NULL, 10);
    if (occupancyState == 0 && errno == EINVAL) {
        Serial.println("Occupancy state is not recognized.");
        return;
    }

    if (occupancyState < 0 || occupancyState > 1)
        Serial.println("err");
    else {
        controller.setOccupancy(occupancyState);
        Serial.println("ack");
    }
}

void windupCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int windup = (int)strtol(valueString, NULL, 10);
    if (windup == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if (windup < 0 || windup > 1)
        Serial.println("err");
    else {
        if (controller.getAntiWindup() != windup) controller.toggleAntiWindup();
        Serial.println("ack");
    }
}

void feedforwardCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int feedforward = (int)strtol(valueString, NULL, 10);
    if (feedforward == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if (feedforward < 0 || feedforward > 1)
        Serial.println("err");
    else {
        if (controller.getFeedforward() != feedforward) controller.toggleFeedforward();
        Serial.println("ack");
    }
}

void feedbackCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int feedback = (int)strtol(valueString, NULL, 10);
    if (feedback == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if (feedback < 0 || feedback > 1)
        Serial.println("err");
    else {
        if (controller.getFeedback() != feedback) controller.toggleFeedback();
        Serial.println("ack");
    }
}

void streamCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *variableString = strsep(&arguments, delimiter);
    if (variableString == NULL) {
        Serial.println("Invalid Stream command.");
        return;
    }
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Stream command.");
        return;
    }

    // Try to convert the variable to a character
    if (strlen(variableString) > 1) {
        Serial.println("Invalid variable for Stream command.");
        return;
    }
    char variableChar = variableString[0];

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Stream command.");
        return;
    }

    switch (variableChar) {
        case 'd':
            if (streamDuty) Serial.println("ack");
            streamDuty = !streamDuty;
            break;
        case 'l':
            if (streamLuminance) Serial.println("ack");
            streamLuminance = !streamLuminance;
            break;
        case 'j':
            if (streamJitter) Serial.println("ack");
            streamJitter = !streamJitter;
            break;
        case 'i':
            if (streamIntegralError) Serial.println("ack");
            streamIntegralError = !streamIntegralError;
            break;
        case 'e':
            if (streamTrackingError) Serial.println("ack");
            streamTrackingError = !streamTrackingError;
            break;
        case 's':
            if (streamSimulator) Serial.println("ack");
            streamSimulator = !streamSimulator;
            break;
        case 'r':
            if (streamReference) Serial.println("ack");
            streamReference = !streamReference;
            break;
        default:
            Serial.println("err");
            return;
    }
}

void bufferCommand(char *arguments) {
    float buffer_copy[60 * 100];
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *variableString = strsep(&arguments, delimiter);
    if (variableString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }

    // Try to convert the variable to a character
    if (strlen(variableString) > 1) {
        Serial.println("Invalid variable for Get command.");
        return;
    }
    char variableChar = variableString[0];

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    switch (variableChar) {
        case 'd':
            controller.getDutyBuffer(buffer_copy);
            for (int i = 0; i < 60 * 100; i++) {
                Serial.print(buffer_copy[i], 6);
                i == 60 * 100 - 1 ? Serial.print('\n') : Serial.print(',');
            }
            break;
        case 'l':
            controller.getIlluminanceBuffer(buffer_copy);
            for (int i = 0; i < 60 * 100; i++) {
                Serial.print(buffer_copy[i], 6);
                i == 60 * 100 - 1 ? Serial.print('\n') : Serial.print(',');
            }
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
    if (subCommand == NULL) {
        Serial.println("Invalid Get command.");
        return;
    }

    // Try to convert the subcommand to a character
    if (strlen(subCommand) > 1) {
        Serial.println("Invalid config subcommand.");
        return;
    }
    char variableChar = subCommand[0];

    // Dump all configuration data:
    // Static gain
    // Taus
    // Controller gain
    if (variableChar == 'd') {
        Serial.print("Static gain: ");
        Serial.println(gain);
        Serial.println("Ascending tau.");
        for (unsigned int i = 0; i < 10; i++) {
            Serial.print("Final lux: ");
            Serial.print(luxAscending[i]);
            Serial.print(" lux");
            Serial.print("\tTau: ");
            Serial.print(tauAscending[i]);
            Serial.println(" s");
        }
        Serial.print('\n');
        Serial.println("Descending tau.");
        for (unsigned int i = 0; i < 10; i++) {
            Serial.print("Final lux: ");
            Serial.print(luxDescending[i]);
            Serial.print(" lux");
            Serial.print("\tTau: ");
            Serial.print(tauDescending[i]);
            Serial.println(" s");
        }

        Serial.print("Proportional gain: ");
        Serial.println(controller.getProportionalGain());

        Serial.print("Integral gain: ");
        Serial.println(controller.getIntegralGain());
    } else if (variableChar == 'p') {
        char *valueString = strsep(&arguments, delimiter);
        if (valueString == NULL) {
            Serial.println("Invalid Duty command.");
            return;
        }
        char *endPointer;
        float proportionalGain = strtof(valueString, &endPointer);
        if (proportionalGain == 0.f && endPointer == valueString) {
            Serial.println("err");
            return;
        }

        if (proportionalGain < 0.f) {
            Serial.println("err");
            return;
        }
        controller.setProportionalGain(proportionalGain);
    } else if (variableChar == 'i') {
        char *valueString = strsep(&arguments, delimiter);
        if (valueString == NULL) {
            Serial.println("Invalid Duty command.");
            return;
        }
        char *endPointer;
        float integralGain = strtof(valueString, &endPointer);
        if (integralGain == 0.f && endPointer == valueString) {
            Serial.println("err");
            return;
        }

        if (integralGain < 0.f) {
            Serial.println("err");
            return;
        }
        if (fequal(integralGain, 0.f) && controller.getAntiWindup() == 1) controller.toggleAntiWindup();

        controller.setIntegralGain(integralGain);
    } else if (variableChar == 'c') {
        controller.turnControllerOff();
        calibrateGain();
        controller.turnControllerOn();
        Serial.println("ack");
    }
}

void simulatorCommand(char *arguments) {
    // Parse the arguments string into its expected two
    // parts
    const char *delimiter = " ";
    char *luminaireString = strsep(&arguments, delimiter);
    if (luminaireString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }
    char *valueString = strsep(&arguments, delimiter);
    if (valueString == NULL) {
        Serial.println("Invalid Duty command.");
        return;
    }

    // Try to convert the luminaire to an index
    int luminaireIndex = (int)strtol(luminaireString, NULL, 10);
    if (luminaireIndex == 0 && errno == EINVAL) {
        Serial.println("Invalid luminaire index for Get command.");
        return;
    }

    // Try to convert the occupancy state into a flag
    int simulator = (int)strtol(valueString, NULL, 10);
    if (simulator == 0 && errno == EINVAL) {
        Serial.println("Windup state is not recognized.");
        return;
    }

    if (simulator < 0 || simulator > 1)
        Serial.println("err");
    else {
        controller.setSimulator(simulator);
        Serial.println("ack");
    }
}

void streamVariables() {
    static unsigned long SampleNum = 0;
    static unsigned long long t = 0;
    sample_t Sample;

    unsigned long CurSampleNum = controller.getSampleNumber();
    if (CurSampleNum > SampleNum) {
        // new sample available
        if (CurSampleNum > SampleNum + 1) {
            // Some samples were lost, estimate the time passed as the target sampling period
            t += (CurSampleNum - SampleNum - 1) * 10000;
        }
        SampleNum = CurSampleNum;

        Sample = controller.getSample();
        t += Sample.dur;

        if (streamLuminance) Serial.printf("s l 0 %.6f %d\n", Sample.L, t);
        if (streamDuty) Serial.printf("s d 0 %.6f %d\n", Sample.u, t);
        if (streamJitter) Serial.printf("s j 0 %.6f %d\n", Sample.dur - SAMPLE_PERIOD_US, t);
        if (streamIntegralError) Serial.printf("s i 0 %.6f %d\n", Sample.IntegralError, t);
        if (streamTrackingError) Serial.printf("s e 0 %.6f %d\n", Sample.TrackingError, t);
        if (streamReference) Serial.printf("s r 0 %.6f %d\n", Sample.Reference, t);
    }
}

// void VariableStream::toggleLumminanceStream() {
//     if (streamLuminance) Serial.println("ack");

//     streamLuminance = !streamLuminance;
// }

// void VariableStream::toggleDutyStream() {
//     if (streamDuty) Serial.println("ack");

//     streamDuty = !streamDuty;
// }

// void VariableStream::toggleJitterStream() {
//     if (streamJitter) Serial.println("ack");

//     streamJitter = !streamJitter;
// }

// void VariableStream::toggleIntegralErrorStream() {
//     if (streamIntegralError) Serial.println("ack");

//     streamIntegralError = !streamIntegralError;
// }

// void VariableStream::toggleTrackingErrorStream() {
//     if (streamTrackingError) Serial.println("ack");

//     streamTrackingError = !streamTrackingError;
// }

// void VariableStream::toggleSimulatorStream() {
//     if (streamSimulator) Serial.println("ack");

//     streamSimulator = !streamSimulator;
// }

// void VariableStream::toggleReferenceStream() {
//     if (streamReference) Serial.println("ack");

//     streamReference = !streamReference;
// }

// void VariableStream::setNewValues() { newValues = true; }

// void VariableStream::streamVariables() {
//     if (!newValues) return;

//     IntFloat intfloat;
//     if (streamLuminance) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'l');
//         intfloat.from = luminanceBuffer.getBegin(1);
//         rp2040.fifo.push(intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     if (streamDuty) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'d');
//         intfloat.from = dutyBuffer.getBegin(1);
//         rp2040.fifo.push(intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     if (streamJitter) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'j');
//         intfloat.from = (float)controller.jitter;
//         rp2040.fifo.push((uint32_t)intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     if (streamIntegralError) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'i');
//         intfloat.from = controller.integralError;
//         rp2040.fifo.push(intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     if (streamTrackingError) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'e');
//         intfloat.from = controller.trackingError;
//         rp2040.fifo.push(intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     if (streamSimulator) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'s');
//         intfloat.from = controller.simulatorValue;
//         rp2040.fifo.push(intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     if (streamReference) {
//         rp2040.fifo.push((uint32_t)'s');
//         rp2040.fifo.push((uint32_t)'r');
//         intfloat.from = controller.reference[controller.occupancy];
//         rp2040.fifo.push(intfloat.to);
//         rp2040.fifo.push((uint32_t)controller.lastTimestamp);
//     }

//     newValues = false;
// }