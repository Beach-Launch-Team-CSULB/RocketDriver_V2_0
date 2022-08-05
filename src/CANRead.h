#ifndef CANREAD_H
#define CANREAD_H

//#include <FlexCAN.h>
#include "ControlCommands.h"
#include "configurationSet.h"

//bool localNodeResetFlag = false;

// header file for the function to call every loop, function returns true if a new message was read
 //vector buffer version
static std::vector<commandMSG> commandMSGvecBuffer{}; 
static std::vector<configMSG> configMSGvecBuffer{}; 

void serialToCAN2Read();

void vectorCommandBufferInitialize(size_t bufferSize);
void vectorConfigBufferInitialize(size_t bufferSize);

void writeVectorBuffer(commandMSG commandStructIn);
void writeVectorBuffer(configMSG configStructIn);

bool readRemoveVectorBuffer(commandMSG& commandStructOut);
bool readRemoveVectorBuffer(configMSG& configMSGOut);

void vectorBufferPrintout();

bool CANread(FlexCAN& CANbus, uint8_t configVerificationKey, bool& NewConfigMessage, Command& CurrentCommand, configMSG& currentConfigMSG, uint8_t propNodeIDIn);
    // passes the CAN bus to be read by reference so methods can be called and buffers emptied as messages are read
    // passes the current command global variable by reference to be updated after the CAN read
    // returns true if a new message was read
    // also tracks if multiple commands have been sent, will update current command with the next oldest command, if one is waiting

bool SerialAsCANread();

#endif