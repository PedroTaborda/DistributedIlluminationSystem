#include "comms.hpp"
#include "parser.hpp"
#include <Wire.h>

Comms *_comms;

Comms::Comms(CommandParser parser) : parser(parser)
{
    Wire.setSDA(SDA_MASTER_PIN);
    Wire.setSCL(SCL_MASTER_PIN);
    Wire.begin();
    Wire1.setSDA(SDA_SLAVE_PIN);
    Wire1.setSCL(SCL_SLAVE_PIN);

    Wire.setTimeout(timeout_ms);
    Wire1.setTimeout(timeout_ms);
    _comms = this;
}

ProcessingResult Comms::processCommand(const char *command)
{
    signed char luminaire_id = parser.getLuminaireId(command);
    int ret = 0;
    bool commandExecutedSuccessfully = false;
    if (!parser.validCommand(command))
        return PROCESSING_INVALID_COMMAND;

    // getLuminaireId returns -1 if no luminaire id was found
    // also, addresses have 7 bits, so all other negative values are invalid
    if (luminaire_id < 0)
        return PROCESSING_INVALID_COMMAND;

    if (luminaire_id == my_id)
    {
        commandExecutedSuccessfully = parser.executeCommand(command);
        return commandExecutedSuccessfully ? PROCESSING_OK : PROCESSING_LOCAL_EXECUTION_FAILURE;
    }

    const char *stripped_command = parser.strip(command);
    Wire.beginTransmission(luminaire_id);
    Wire.write(stripped_command);
    ret = Wire.endTransmission();
    if (ret == 0){
        return PROCESSING_OK;
    }
    else if (ret == 5){
        return PROCESSING_I2C_TIMEOUT;
    }
    else{
        return PROCESSING_I2C_OTHER_ERROR;
    }
}

bool Comms::joinNetwork()
{
    int ret;
    for (int my_potential_addr = my_id + addr_offset; my_potential_addr < 128; my_potential_addr++)
    {
        do{
            Wire.beginTransmission(my_potential_addr);
            ret = Wire.endTransmission(false);
        } while(ret == 4);

        if (!ret)
        {
            Serial.printf("Addr %d occupied\n", my_potential_addr);
            Serial.printf("Response: %d\n", ret);
        }
        else
        {
            Serial.printf("Addr %d unoccupied\n", my_potential_addr);
            my_id = my_potential_addr - addr_offset;
            Wire1.begin(my_potential_addr);
            Wire1.onReceive([](int i){ _comms->onReceive(i); });
            Wire1.onRequest([](){ _comms->onRequest(); });
            return true;
        }

    }
    return false;
}

void Comms::onReceive(int signed bytesReceived)
{
    MSG_TYPE msg_type = (MSG_TYPE) Wire.read();
    error = true;
    sprintf(error_msg, "Received %d bytes. Message type: %d\n", bytesReceived, msg_type);

    switch (msg_type)
    {
    /*case MSG_TYPE_WAKEUP_REQUEST:
        if (bytesReceived != 1)
        {
            error = true;
            sprintf(error_msg, "Invalid wakeup request message received");
            break;
        }
        requestedDataSize = 1;
        requestedData[0] = 1;
        break;
    */
    default:
        break;
    }
    if (requestedDataSize > requestedDataBufferSize)
    {
        error = true;
        sprintf(error_msg, "Requested data buffer overflow");
    }
}

void Comms::onRequest()
{
    Wire.write(1);
    /*for (int i = 0; i < requestedDataSize; i++)
    {
        Wire.write(requestedData[i]);
    }*/
}

void Comms::flushError(){
    if (error)
        Serial.printf("%s\n", error_msg);
    error = false;
}

void Comms::eventLoop()
{
    flushError();
}
