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

struct calibrationCoefs
{
    float m_coef;
    float b_coef;
};

struct ALARACalibrations
{
    calibrationCoefs HP1_Calibration;
    calibrationCoefs HP2_Calibration;
    calibrationCoefs HP3_Calibration;
    calibrationCoefs HP4_Calibration;
    calibrationCoefs HP5_Calibration;
    calibrationCoefs HP6_Calibration;
    calibrationCoefs HP7_Calibration;
    calibrationCoefs HP8_Calibration;
    calibrationCoefs HP9_Calibration;
    calibrationCoefs HP10_Calibration;
    calibrationCoefs rail3V3Calibration;
    calibrationCoefs rail5V0Calibration;
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
    uint32_t NOR1_size;
    uint32_t NOR2_size;
    uint32_t NOR3_size;
    uint32_t NOR4_size;
    uint32_t NOR5_size;
    uint32_t NOR6_size;
    uint32_t NOR7_size;
    uint32_t NOR8_size;
    ALARACalibrations boardCalStruct;
};

void lookupALARASNmap(ALARASN& thisALARA, uint8_t ALARAnodeID);

#endif