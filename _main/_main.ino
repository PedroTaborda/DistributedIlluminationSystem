#include <Arduino.h>
#include <EEPROM.h>

#include <pico/stdlib.h>
#include <hardware/dma.h>

#include "calibration.hpp"
#include "controller.hpp"
#include "globals.hpp"
#include "interface.hpp"
#include "utilities.hpp"
#include "comms.hpp"
#include "parser.hpp"

// Initialization of global variables

float gammaFactor = 0.f;
float gain = 0.f;
float ambientIlluminance = 0.f;
double tauAscending[10], tauDescending[10];
double luxAscending[10], luxDescending[10];

volatile Controller controller;
repeating_timer timerStruct;

volatile Buffer<float, 60*100> luminanceBuffer;
volatile Buffer<float, 60*100> dutyBuffer;
bool streamLuminanceBuffer = false;
bool streamDutyBuffer = false;
VariableStream streamer;
volatile unsigned long lastTimestamp = 0;
volatile double energy = 0.f;
volatile double visibilityAccumulator = 0.f;
volatile double flickerAccumulator = 0.f;
volatile double previousFlicker = 0.f;
volatile double previousLux = 0.f;
volatile unsigned long sampleNumber = 0;

alarm_pool_t* core1AlarmPool;

bool bufferLock = false;
int dmaChannel;
float copyBuffer[60*100];
int currentHead = 0;

int myID = 0;

IntFloat intfloat0, intfloat1;

CommandParser parser(
    (Command[]){
    {'a', "<float>", "reply is number + 2", floatArgCom([](float f){printI2C("a %d %f", myID, f+2)}), NULL},
    {'h', "", "help", noArgCom(help), NULL},
    {'g', "", "get command", NULL, (Command[]){
        {'r', "", "reference", printfCom("reference :)"), NULL},
        {'\0', "", "", NULL, NULL}}},
    {'p', "", "ack", noArgCom([](){ACK}), NULL},
    {'t', "", "time", printfCom("t %d %lu", myID, millis()), NULL},
    {'\0', "", "", NULL, NULL}
}
);

Comms comms(parser);

// CORE 0 is in charge of communications with the computer
// CORE 1 handles the controller

void setup() {
    // Initialize Serial protocol
    Serial.begin();
    comms.init();
    comms.joinNetwork();

    while(calibrator.waiting()) ;

    if(comms.my_id == 0x0)
        comms.calibrateNetwork();
    /*
        // Pause the other core and read constants from the EEPROM
        rp2040.idleOtherCore();

        EEPROM.begin(4096);
        int EEPROMAddress = 0;
        EEPROM.get(EEPROMAddress, gammaFactor);
        EEPROMAddress += sizeof(float);

        for(int tauCounter = 0; tauCounter < 10; tauCounter++) {
            float lux, tau;
            EEPROM.get(EEPROMAddress, lux);
            EEPROMAddress += sizeof(float);
            EEPROM.get(EEPROMAddress, tau);
            EEPROMAddress += sizeof(float);

            luxAscending[tauCounter] = lux;
            tauAscending[tauCounter] = tau;
        }

        for(int tauCounter = 0; tauCounter < 10; tauCounter++) {
            float lux, tau;
            EEPROM.get(EEPROMAddress, lux);
            EEPROMAddress += sizeof(float);
            EEPROM.get(EEPROMAddress, tau);
            EEPROMAddress += sizeof(float);

            luxDescending[tauCounter] = lux;
            tauDescending[tauCounter] = tau;
        }

        EEPROM.end();
        // Resume the other core
        rp2040.resumeOtherCore();
    */
}

void loop() {
    if(Serial.available() > 0)
    {
        parseSerial(comms);
    }
    comms.eventLoop();
    /*
        uint32_t queueResult;
        if(rp2040.fifo.pop_nb(&queueResult)) {
            char command = (char) queueResult;
            char variable;
            float variableValue;
            unsigned long timeStamp;
            switch(command) {
            case 'B':
                queueResult = rp2040.fifo.pop();
                variable = (char) queueResult;
                Serial.print("B "); Serial.print(variable); Serial.print(" 0 ");
                for(int i = 0; i < 60*100; i++) {
                    Serial.print(copyBuffer[(currentHead + i) % (60 * 100)], 6);
                    i == 60*100 - 1 ? Serial.print('\n') : Serial.print(',');
                }
                bufferLock = false;
                streamDutyBuffer = false;
                streamLuminanceBuffer = false;
                break;
            case 's':
                queueResult = rp2040.fifo.pop();
                variable = (char) queueResult;
                queueResult = rp2040.fifo.pop();
                intfloat0.to = queueResult;
                variableValue = intfloat0.from;
                queueResult = rp2040.fifo.pop();
                timeStamp = (unsigned long) queueResult;

                Serial.print("s "); Serial.print(variable); Serial.print(" 0 ");
                Serial.print(variableValue, 6); Serial.print(' '); Serial.println(timeStamp / 1000);
                break;
            }
        }
    */
}
/*
void setup1() {
    // Initialize the ADC and DAC settings
    analogReadResolution(12);
    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(DAC_RANGE);

    // Calibrate the system's static gain
    calibrateGain();

    dmaChannel = dma_claim_unused_channel(true);
    core1AlarmPool = alarm_pool_create(1, 1);

    // Setup the controller
    controller.setup(0.01, 0.05, &timerStruct);
}

void loop1() {
    uint32_t queueResult;
    if(rp2040.fifo.pop_nb(&queueResult)) {
        char command = (char) queueResult;

        switch(command) {
        case 'd':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            intfloat1.to = queueResult;
            SAFE_I_ACCESS(
                controller.setDutyCycle(intfloat1.from);
            )
            break;
        case 'r':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            intfloat1.to = queueResult;
            SAFE_I_ACCESS(
                controller.setReference(intfloat1.from);
            )
            break;
        case 'o':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            SAFE_I_ACCESS(
                controller.setOccupancy((int) queueResult);
            )
            break;
        case 'a':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            SAFE_I_ACCESS(
                if(controller.getAntiWindup() != (int) queueResult)
                    controller.toggleAntiWindup();
            )
            break;
        case 'w':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            SAFE_I_ACCESS(
                if(controller.getFeedforward() != (int) queueResult)
                    controller.toggleFeedforward();
            )
            break;
        case 'b':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            SAFE_I_ACCESS(
                if(controller.getFeedback() != (int) queueResult)
                    controller.toggleFeedback();
            )
            break;
        case 'c':
            SAFE_I_ACCESS(
                calibrateGain();
            )
            rp2040.fifo.push((uint32_t) 0x12345678);
            break;
        case 'm':
            // Wait until the value arrives
            queueResult = rp2040.fifo.pop();
            SAFE_I_ACCESS(
                controller.setSimulator((int) queueResult);
            )
            break;
        default:
            break;
        }
    }

    if(!bufferLock && streamLuminanceBuffer) {
        SAFE_I_ACCESS(
            float *src = (float *)luminanceBuffer._getBufferLocation();
            float *dst = copyBuffer;
            currentHead = luminanceBuffer.getCurrentHead();

            dma_channel_config config = dma_channel_get_default_config(dmaChannel);
            channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
            channel_config_set_read_increment(&config, true);
            channel_config_set_write_increment(&config, true);

            dma_channel_configure(
                dmaChannel,    // Channel to be configured
                &config,       // The configuration we just created
                dst,           // The initial write address
                src,           // The initial read address
                60*100,        // Number of transfers; in this case each is 1 byte.
                true           // Start immediately.
            );

            dma_channel_wait_for_finish_blocking(dmaChannel);
        )

        bufferLock = true;
        rp2040.fifo.push((uint32_t) 'B');
        rp2040.fifo.push((uint32_t) 'l');
    }

    if(!bufferLock && streamDutyBuffer) {
        SAFE_I_ACCESS(
            float *src = (float *)dutyBuffer._getBufferLocation();
            float *dst = copyBuffer;
            currentHead = dutyBuffer.getCurrentHead();

            dma_channel_config config = dma_channel_get_default_config(dmaChannel);
            channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
            channel_config_set_read_increment(&config, true);
            channel_config_set_write_increment(&config, true);

            dma_channel_configure(
                dmaChannel,    // Channel to be configured
                &config,       // The configuration we just created
                dst,           // The initial write address
                src,           // The initial read address
                60*100,        // Number of transfers; in this case each is 1 byte.
                true           // Start immediately.
            );

            dma_channel_wait_for_finish_blocking(dmaChannel);
        )

        bufferLock = true;
        rp2040.fifo.push((uint32_t) 'B');
        rp2040.fifo.push((uint32_t) 'd');
    }

    streamer.streamVariables();
}
*/
