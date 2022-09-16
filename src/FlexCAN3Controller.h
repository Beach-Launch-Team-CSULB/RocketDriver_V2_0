#ifndef FLEXCAN3CONTROLLER_H
#define FLEXCAN3CONTROLLER_H

#include <Arduino.h>
#include <FlexCAN.h>
#include "ControlFunctions.h" //need to shift the include tree into generic control function includes probably but for now leave it just PasaBang
#include <array>
using std::array;
#include <vector>
using std::vector;



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
    CAN_message_t packedSensorCAN2; //CAN2 frame format for up to 3 converted ADC reads

    uint8_t numberSensors = 0;      //tracks how many sensor reads are going into the frame

    uint8_t sensorID[3];
    uint16_t sensorConvertedValue[3];
    uint32_t sensorTimestampMicros[3];
    uint32_t fractionalTimestamp; //need to chop into 18 bit IDB field
    uint32_t timestampTolerance;     //range in micros from the timestamp a value can be

    uint8_t frameTotalBits;     //calculated bits for estimating bus load

};

struct ALARA_TankControllermsgs
{
    CAN_message_t controllerStateReportCanFrame;
    
    elapsedMillis quasistaticUpdateTimer;
    uint32_t quasistaticSendTime = 2000;
    bool quasistaticSendBool;
    //std::vector<CAN_message_t> tankControllerCAN2Frames; //CAN2 frame format 
    uint8_t controllerID;
    uint16_t controllerStateReportID;
    //float controllerFloatValuesArray[12];
/*     union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t uint32Value;             //unsigned 32 bit
        uint8_t uint8Value4X[4];
        float floatValue = 0;
    };
 */
    uint32_t frameTotalBits;     //calculated bits for estimating bus load

};

class FlexCan3Controller
{
    private:
        ALARA_HP_CAN2report nodeObjectIDReportStruct;
        ALARA_HP_CAN2report nodeObjectStateReportStruct;
        ALARA_RawSensorReadmsg sensorRawReadStruct_current;
        ALARA_ConvertedSensorReadmsg sensorConvertedReadStruct_current;
/*         ALARA_TankControllermsgs fuelTankPressControllerReportsStruct;
        ALARA_TankControllermsgs loxTankPressControllerReportsStruct;
 */        
        ALARA_TankControllermsgs tankPressControllerReportsStruct;

        elapsedMillis propNodeStateReportTimer;
        elapsedMillis convertedValueUpdateTimer;
        elapsedMicros rawValueUpdateTimer;
        elapsedMillis highPowerObjectIDmsgTimer;
        elapsedMillis highPowerStatemsgTimer;
        elapsedMillis nodeSystemTimemsgTimer;
        elapsedMillis AutoSequenceReportTimer;
        uint32_t propNodeStateReportRateMillis = 1000;   // in Millis default value
        uint32_t highPowerObjectIDRateMillis = 10000;   // in Millis default value
        //uint32_t highPowerObjectIDRateHzDenominator = 2;   // used to get less than 1Hz value via fraction
        uint32_t highPowerStateReportRateMillis = 250;   // in Millis default value
        uint32_t convertedSendRateMillis = 125;          // in Millis default value
        uint32_t rawSendRateMicros = 250;          // in Micros default value
        uint32_t nodeSystemTimemsgSendRateMillis = 1000;          // in Millis default value
        bool objectIDmsgs = false;  //bool for if the msg has been generated yet
        bool objectIDmsgsSendBool = false;  //bool for if the msg should be sent based on controller setting the bool

        bool externalStateChange = false;    //bool that can be used to send all infrequent messages at a state change
        
        elapsedMicros convertedSendTimeoutbreak;
        uint32_t convertedSendTimeoutMicros = 500;

    public:
        //assemble message functions
        void generatePropNodeStateReport(FlexCAN& CANbus,  VehicleState& currentState, MissionState& currentMissionState, Command& currentCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const uint8_t& propulsionNodeIDIn);
        void generateHPObjectIDmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn);
        void generateHPObjectStateReportmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn);
        bool generateRawSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<ALARAHP_SENSOR*, NUM_HPSENSORS>& HPsensorArray, const uint8_t& propulsionNodeIDIn);
        bool generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<ALARAHP_SENSOR*, NUM_HPSENSORS>& HPsensorArray, const uint8_t& propulsionNodeIDIn);
        bool generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<ALARAHP_SENSOR*, NUM_HPSENSORS>& HPsensorArray, const uint8_t& propulsionNodeIDIn);
        void generateTankControllermsgs(FlexCAN& CANbus, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const uint8_t& propulsionNodeIDIn);
        void generateAutoSequenceUpdatemsg(FlexCAN& CANbus, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const uint8_t& propulsionNodeIDIn);
        void generateEngineControllermsgs(FlexCAN& CANbus, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, const uint8_t& propulsionNodeIDIn);
        void generateFluidSimmsgs(FlexCAN& CANbus, FluidSystemSimulation& fluidSim, const uint8_t& propulsionNodeIDIn);

        void writeObjectByteArray(uint8_t byteArray[10], CAN_message_t& msgIn, uint16_t IDA);
        void writeNodeStateReportByteArray(uint8_t byteArray[8], CAN_message_t& msgIn, uint16_t IDA);
        void nodeSystemTimemsg(FlexCAN& CANbus);
        // External state change bool set function
        void setExternalStateChange(bool stateChangeIn){externalStateChange = stateChangeIn;}
        //Controller loop function
        void controllerTasks(FlexCAN& CANbus, VehicleState& currentState, MissionState& currentMissionState, Command& currentCommand, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<ALARAHP_SENSOR*, NUM_HPSENSORS>& HPsensorArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, FluidSystemSimulation& fluidSim, const uint8_t& propulsionNodeIDIn);
};

#endif