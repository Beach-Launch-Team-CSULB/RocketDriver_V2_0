#ifndef FLEXCAN3CONTROLLER_H
#define FLEXCAN3CONTROLLER_H

#include <Arduino.h>
#include <FlexCAN.h>
#include "ControlFunctionsPasaBang.h" //need to shift the include tree into generic control function includes probably but for now leave it just PasaBang



struct ALARA_HP_CAN2report
{
    //CAN2 format needs to split across two CAN frames
    // messages that are just the object IDs on this ALARA for the HP channels, not sent frequently
    CAN_message_t objectIDmsg;
    uint8_t objectIDByteArray[10];
    //reporting messages for the HP object state info
    CAN_message_t reportmsg1;
    

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
        ALARA_HP_CAN2report nodeObjectIDReportStruct;
        ALARA_HP_CAN2report nodeObjectStateReportStruct;
        ALARA_RawSensorReadmsg sensorRawReadStruct_current;
        ALARA_ConvertedSensorReadmsg sensorConvertedReadStruct_current;
        
        elapsedMillis convertedValueUpdateTimer;
        elapsedMillis highPowerObjectIDmsgTimer;
        elapsedMillis highPowerStatemsgTimer;
        elapsedMillis nodeSystemTimemsgTimer;

        uint16_t highPowerObjectIDRateHz = 1;   // in Hz default value
        uint16_t highPowerObjectIDRateHzDenominator = 2;   // used to get less than 1Hz value via fraction
        uint16_t highPowerStateReportRateHz = 2;   // in Hz default value
        uint16_t convertedSendRateHz = 10;          // in Hz default value
        bool objectIDmsgs = false;  //bool for if the msg has been generated yet

    public:

        //assemble message functions
        void generateHPObjectIDmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn);
        void generateHPObjectStateReportmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn);
        void generateRawSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn);
        void generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn);
        void writeObjectByteArray(uint8_t byteArray[10], CAN_message_t& msgIn, uint16_t IDA);
        void nodeSystemTimemsg(FlexCAN& CANbus);

        //Controller loop function
        void controllerTasks(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn);
};

#endif