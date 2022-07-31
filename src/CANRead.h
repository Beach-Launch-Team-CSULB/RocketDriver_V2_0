#ifndef CANREAD_H
#define CANREAD_H

//#include <FlexCAN.h>
#include "ControlCommands.h"
#include "configurationSet.h"

//bool localNodeResetFlag = false;

// header file for the function to call every loop, function returns true if a new message was read

void serialToCAN2Read();



bool CANread(FlexCAN& CANbus, uint8_t configVerificationKey, bool& NewConfigMessage, Command& CurrentCommand, configMSG& currentConfigMSG, uint8_t propNodeIDIn);
    // passes the CAN bus to be read by reference so methods can be called and buffers emptied as messages are read
    // passes the current command global variable by reference to be updated after the CAN read
    // returns true if a new message was read
    // also tracks if multiple commands have been sent, will update current command with the next oldest command, if one is waiting
    // NO SUPPORT FOR READING IDS MUST BE ADDED LATER



#endif