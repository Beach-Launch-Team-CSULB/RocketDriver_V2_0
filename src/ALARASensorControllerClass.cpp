#include "ALARASensorControllerClass.h"
#include <Arduino.h>

ALARAV2SensorController::ALARAV2SensorController(uint32_t setControllerID, uint8_t setControllerNodeID, bool setNodeIDCheck, bool setBNO055_active, bool setBMI085_active, bool setKX134_1211_active, bool setSAM_M8Q_GPS_active, bool setMS5607_active, bool setALARA_VINRail_active, bool setALARA_5VRail_active, bool setALARA_3V3Rail_active)
                         : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, nodeIDCheck{setNodeIDCheck}, BNO055_active{setBNO055_active}, BMI085_active{setBMI085_active}, KX134_1211_active{setKX134_1211_active}, SAM_M8Q_GPS_active{setSAM_M8Q_GPS_active}, MS5607_active{setMS5607_active}, ALARA_VINRail_active{setALARA_VINRail_active}, ALARA_5VRail_active{setALARA_5VRail_active}, ALARA_3V3Rail_active{setALARA_3V3Rail_active}
{
    // Instantiation stuff?
}

void ALARAV2SensorController::begin()
{
    if (nodeIDCheck)
    {
        // setup stuff?
    }
}

/* void ALARAV2SensorController::setALARAHPSensors(ALARAHP_SENSOR* setHP1, ALARAHP_SENSOR* setHP2, ALARAHP_SENSOR* setHP3,ALARAHP_SENSOR* setHP4, ALARAHP_SENSOR* setHP5, ALARAHP_SENSOR* setHP6,ALARAHP_SENSOR* setHP7, ALARAHP_SENSOR* setHP8, ALARAHP_SENSOR* setHP9, ALARAHP_SENSOR* setHP10)
{
    // won't work, these objects are only inside this scope. I was hoping to dodge shitty constructor here but odn't see how
    ALARAHP_SENSOR &HP1 = *setHP1;
    ALARAHP_SENSOR &HP2 = *setHP2;
    ALARAHP_SENSOR &HP3 = *setHP3;
    ALARAHP_SENSOR &HP4 = *setHP4;
    ALARAHP_SENSOR &HP5 = *setHP5;
    ALARAHP_SENSOR &HP6 = *setHP6;
    ALARAHP_SENSOR &HP7 = *setHP7;
    ALARAHP_SENSOR &HP8 = *setHP8;
    ALARAHP_SENSOR &HP9 = *setHP9;
    ALARAHP_SENSOR &HP10 = *setHP10;
}
 */

void ALARAV2SensorController::resetTimer()
{
    //timer = 0;
}

void ALARAV2SensorController::stateOperations()
{
    switch (state)
    {
    case ALARAV2SensorControllerState::Passive:
        sensorStateInternal = SensorState::Off;
        sensorStateALARAHP = SensorState::Off;
        break;
    case ALARAV2SensorControllerState::Standby:
        sensorStateInternal = SensorState::Slow;
        sensorStateALARAHP = SensorState::Slow;
        break;
    case ALARAV2SensorControllerState::Active:
        sensorStateInternal = SensorState::Fast;
        sensorStateALARAHP = SensorState::Fast;
        break;
    case ALARAV2SensorControllerState::InternalOnly:
        sensorStateInternal = SensorState::Fast;
        sensorStateALARAHP = SensorState::Off;
        break;
    
    default:
        break;
    }
}

void ALARAV2SensorController::ALARAconfigurationSensorSet(ALARASN& thisALARA)
{
    setBNO055_active(thisALARA.BNO055_present);
    setBMI085_active(thisALARA.BMI085_present);
    setKX134_1211_active(thisALARA.KX134_1211_present);
    setSAM_M8Q_GPS_active(thisALARA.SAM_M8Q_GPS_present);
    setMS5607_active(thisALARA.SAM_M8Q_GPS_present);

    // This section is for board rev dependant sensors NOT listed explicitly in configurations
/*     if (static_cast<uint8_t>(thisALARA.boardRev) == 1)          // ALARA V2_0
    {
        ALARA_5VRail_active = true;
        ALARA_3V3Rail_active = true;
    }
    if (static_cast<uint8_t>(thisALARA.boardRev) == 2)          // ALARA V2_1
    {
        ALARA_VINRail_active = true;
        ALARA_5VRail_active = true;
        ALARA_3V3Rail_active = true;
    } */
// if below works then delete the staticcast version above
    if (thisALARA.boardRev == ALARAversion::V2_0)          // ALARA V2_0
    {
        ALARA_5VRail_active = true;
        ALARA_3V3Rail_active = true;
    }
    if (thisALARA.boardRev == ALARAversion::V2_1)          // ALARA V2_1
    {
        ALARA_VINRail_active = true; //Did this make it to V2_1 production??
        ALARA_5VRail_active = true;
        ALARA_3V3Rail_active = true;
    }

}