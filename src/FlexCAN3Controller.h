#ifndef FLEXCAN3CONTROLLER_H
#define FLEXCAN3CONTROLLER_H

#include <Arduino.h>
#include <FlexCAN.h>
#include "ControlFunctionsPasaBang.h" //need to shift the include tree into generic control function includes probably but for now leave it just PasaBang



struct ALARA_HP_CAN2report
{
    //CAN2 format needs to split across two CAN frames
    // messages that are just the object IDs on this ALARA for the HP channels, not sent frequently
    CAN_message_t objectIDmsg1;
    CAN_message_t objectIDmsg2;
    //reporting messages for the HP object state info
    CAN_message_t reportmsg1;
    CAN_message_t reportmsg2;

};

struct ALARA_RawSensorReadmsg
{
    CAN_message_t packedSensorCAN2; //CAN2 frame format for up to 3 raw ADC reads

    uint8_t numberSensors = 0;      //tracks how many sensor reads are going into the frame

    uint8_t sensorID[3];
    uint16_t sensorRawValue[3];
    uint32_t sensorTimestampMicros[3];
    uint32_t fractionalTimestamp; //need to chop into 18 bit IDB field
    uint16_t timestampTolerance;     //range in micros from the timestamp a value can be

    uint8_t frameTotalBits;     //calculated bits for estimating bus load

};

struct ALARA_ConvertedSensorReadmsg
{
    //elapsedMillis convertedValueUpdateTimer;
    CAN_message_t packedSensorCAN2; //CAN2 frame format for up to 3 raw ADC reads

    uint8_t numberSensors = 0;      //tracks how many sensor reads are going into the frame

    uint8_t sensorID[3];
    uint16_t sensorConvertedValue[3];
    uint32_t sensorTimestampMicros[3];
    uint32_t fractionalTimestamp; //need to chop into 18 bit IDB field
    uint32_t timestampTolerance;     //range in micros from the timestamp a value can be

    uint8_t frameTotalBits;     //calculated bits for estimating bus load

};

class FlexCan3Controller
{
    private:
        ALARA_HP_CAN2report nodeReportStruct;
        ALARA_RawSensorReadmsg sensorRawReadStruct_current;
        ALARA_ConvertedSensorReadmsg sensorConvertedReadStruct_current;
        elapsedMillis convertedValueUpdateTimer;

    public:

        //assemble object ID message function
        void generateObjectIDmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn);
        void generateRawSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn);
        void generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn, uint32_t convertedSendRateHz);


};

#endif