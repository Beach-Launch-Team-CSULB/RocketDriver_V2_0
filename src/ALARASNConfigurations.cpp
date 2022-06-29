#include "ALARASNConfigurations.h"
#include <iostream>
#include <unordered_map>

// struct ALARASN format
//  {ALARAaddress, propulsionSysNodeID, boardRev, BNO055_present, BMI085_present, KX134_1211_present, SAM_M8Q_GPS_present, MS5607_present}

// map that contains board config information for all ALARAs built
// Add all new ALARAs built here - USE SEQUENTIAL SERIAL NUMBERING, REGARDLESS OF ALARA BOARD REV
// Assign board roles and use related config info
// ----- ALARA ADDRESSING ----- //
// V1, V2_0:    0-15 possible address space based on 4 addressing pins
// V2_1:        0-255 possible address space based on 8 addressing pins

/* void createALARASNmap()
{
    std::unordered_map<uint16_t, ALARASN> ALARASNmap
    {
    // map key = ALARA address
    {1, ALARASN {1, 3, ALARAversion::V1, true, false, true, false, true}},
    {2, ALARASN {2, 3, ALARAversion::V1, true, false, true, false, true}},
    {3, ALARASN {3, 2, ALARAversion::V2_0, false, true, true, true, true}},
    {4, ALARASN {4, 3, ALARAversion::V2_1, false, true, true, true, true}},
    };
}; */

std::unordered_map<uint16_t, ALARASN> ALARASNmap
{
// map key = ALARA address
{1, ALARASN {1, 3, ALARAversion::V1, true, false, true, false, true}},
{2, ALARASN {2, 3, ALARAversion::V1, true, false, true, false, true}},
{3, ALARASN {3, 2, ALARAversion::V2_0, false, true, true, true, true}},
{4, ALARASN {4, 3, ALARAversion::V2_1, false, true, true, true, true}},
};

void lookupALARASNmap(ALARASN& thisALARA, uint8_t ALARANodeIDIn)
{
    thisALARA = ALARASNmap[ALARANodeIDIn];
};