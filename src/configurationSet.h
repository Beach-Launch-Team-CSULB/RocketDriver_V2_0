#ifndef CONFIGURATIONSET_H  
#define CONFIGURATIONSET_H

#include <Arduino.h>

//I should probably change to using typedef for my arg type

struct configMSG
{
    // CAN configuration message format
    uint8_t TargetObjectID;             // object ID for the setting
    uint8_t ObjectSettingID;            // specific setting ID within the object to update
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t uint32Value;             //unsigned 32 bit
        int32_t int32Value;
        uint16_t uint16Value;
        int16_t int16Value;
        uint8_t uint8Value;
        int8_t int8Value;
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