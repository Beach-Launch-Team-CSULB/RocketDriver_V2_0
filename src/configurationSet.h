#ifndef CONFIGURATIONSET_H  
#define CONFIGURATIONSET_H

#include <Arduino.h>
#pragma once



//I should probably change to using typedef for my arg type
struct configMSG
{
    // CAN configuration message format
    uint8_t verificationKey;             // verification key that must match for the message to be considered valid, protection layer from false config messages
    uint8_t TargetObjectID;             // object ID for the setting
    uint8_t ObjectSettingID;            // specific setting ID within the object to update
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t uint32Value;             //unsigned 32 bit
        int32_t int32Value;
        uint16_t uint16Value;
        uint16_t uint16Value2X[2];
        int16_t int16Value;
        int16_t int16Value2X[2];
        uint8_t uint8Value;
        uint8_t uint8Value4X[4];
        int8_t int8Value;
        int8_t int8Value4X[4];

        float floatValue;
    };
    
};

struct configSet
{
    uint8_t TargetObjectID;             // object ID for the setting
    uint8_t ObjectSettingID;            // specific setting ID within the object to update
    uint8_t settingArgType;             // 8 bit int to map my possible arg types for ints/floats
};

#endif