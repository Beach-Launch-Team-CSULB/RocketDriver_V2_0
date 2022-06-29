#ifndef ALARASNCONFIGURATION_H
#define ALARASNCONFIGURATION_H
#pragma once
#include <Arduino.h>


// Add any board revisions this enum list
enum class ALARAversion
{
V1,
V2_0,
V2_1,
};

// Add any board characteristics or configuration parameters to the following struct
// Update the map for all SNs whenever adding a parameter
struct ALARASN
{
    uint16_t ALARAaddress;
    uint8_t propulsionSysNodeID;
    ALARAversion boardRev;
    bool BNO055_present;
    bool BMI085_present;
    bool KX134_1211_present;
    bool SAM_M8Q_GPS_present;
    bool MS5607_present;
};

void lookupALARASNmap(ALARASN& thisALARA, uint8_t ALARAnodeID);

#endif