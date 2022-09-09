#include "FlexCAN3Controller.h"

// CAN2 bit overhead precalculated values, row 0 is number of data bytes, row 1 is bits with standard ID only, row 2 is bits with extended ID 
// Values are likely WRONG to some degree, my calculatons don't match wiki stated values but good enough for rough bus load estimation
uint8_t standardIDCAN2TotalBits[3][9] = {{0,1,2,3,4,5,6,7,8},{52,62,72,82,92,102,112,122,132},{77,87,97,107,117,127,137,147,157}};
const uint32_t max18bitvalue = 262143;  //preset as constant to not calculate it

void FlexCan3Controller::writeObjectByteArray(uint8_t byteArray[10], CAN_message_t& msgIn, uint16_t IDA)
{
    msgIn.len = 8; //always a full frame this format
    msgIn.flags.extended = 1;
    msgIn.flags.remote = 0;
    msgIn.id = IDA + (byteArray[0] << 12) + (byteArray[1] << 20);   //standard ID plus pack first two bytes into back of extendedID field
    for (size_t i = 0; i < 8; i++)
    {
        msgIn.buf[i] = byteArray[i+2];  //pack the back 8 elements of byte array into normal CAN2 bytes
    }
    //return msgIn;
}

void FlexCan3Controller::writeNodeStateReportByteArray(uint8_t byteArray[8], CAN_message_t& msgIn, uint16_t IDA)
{
    msgIn.len = 8; //always a full frame this format
    msgIn.flags.extended = 0;
    msgIn.flags.remote = 0;
    msgIn.id = IDA;   //standard ID plus pack first two bytes into back of extendedID field
    for (size_t i = 0; i < 8; i++)
    {
        msgIn.buf[i] = byteArray[i];  //pack the back 8 elements of byte array into normal CAN2 bytes
    }
}

CAN_message_t writeDouble4ByteDataCAN2Frame(uint16_t msgIDIn, float float1In, float float2In)
{
    CAN_message_t frameToPackage;
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t func_uint32Value;             //unsigned 32 bit
        uint8_t func_uint8Value4X[4];
        float func_floatValue = 0;
    };

    frameToPackage.flags.extended = 0;
    frameToPackage.flags.remote = 0;
    frameToPackage.id = msgIDIn;
    frameToPackage.len = 8;
    func_floatValue = float1In;
    frameToPackage.buf[0] = func_uint8Value4X[3];
    frameToPackage.buf[1] = func_uint8Value4X[2];
    frameToPackage.buf[2] = func_uint8Value4X[1];
    frameToPackage.buf[3] = func_uint8Value4X[0];
    func_floatValue = float2In;
    frameToPackage.buf[4] = func_uint8Value4X[3];
    frameToPackage.buf[5] = func_uint8Value4X[2];
    frameToPackage.buf[6] = func_uint8Value4X[1];
    frameToPackage.buf[7] = func_uint8Value4X[0];

    return frameToPackage;
}

CAN_message_t writeDouble4ByteDataCAN2Frame(uint16_t msgIDIn, float float1In)
{
    CAN_message_t frameToPackage;
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t func_uint32Value;             //unsigned 32 bit
        uint8_t func_uint8Value4X[4];
        float func_floatValue = 0;
    };

    frameToPackage.flags.extended = 0;
    frameToPackage.flags.remote = 0;
    frameToPackage.id = msgIDIn;
    frameToPackage.len = 4;
    func_floatValue = float1In;
    frameToPackage.buf[0] = func_uint8Value4X[3];
    frameToPackage.buf[1] = func_uint8Value4X[2];
    frameToPackage.buf[2] = func_uint8Value4X[1];
    frameToPackage.buf[3] = func_uint8Value4X[0];

    return frameToPackage;
}

CAN_message_t writeDouble4ByteDataCAN2Frame(uint16_t msgIDIn, uint32_t uint32_t1In, uint32_t uint32_t2In)
{
    CAN_message_t frameToPackage;
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t func_uint32Value;             //unsigned 32 bit
        uint8_t func_uint8Value4X[4];
        float func_floatValue = 0;
    };

    frameToPackage.flags.extended = 0;
    frameToPackage.flags.remote = 0;
    frameToPackage.id = msgIDIn;    //I should add bit chopping to make sure it doesn't push into ExtendedID bits
    frameToPackage.len = 8;
    func_uint32Value = uint32_t1In;
    frameToPackage.buf[0] = func_uint8Value4X[3];
    frameToPackage.buf[1] = func_uint8Value4X[2];
    frameToPackage.buf[2] = func_uint8Value4X[1];
    frameToPackage.buf[3] = func_uint8Value4X[0];
    func_uint32Value = uint32_t2In;
    frameToPackage.buf[4] = func_uint8Value4X[3];
    frameToPackage.buf[5] = func_uint8Value4X[2];
    frameToPackage.buf[6] = func_uint8Value4X[1];
    frameToPackage.buf[7] = func_uint8Value4X[0];
    
    return frameToPackage;
}

CAN_message_t writeDouble4ByteDataCAN2Frame(uint16_t msgIDIn, int32_t int32_t1In, int32_t int32_t2In)
{
    CAN_message_t frameToPackage;
    union                       // union for storing bus bytes and pulling as desired value format
    {
        int32_t func_int32Value;             //signed 32 bit
        uint8_t func_uint8Value4X[4];
        float func_floatValue = 0;
    };

    frameToPackage.flags.extended = 0;
    frameToPackage.flags.remote = 0;
    frameToPackage.id = msgIDIn;    //I should add bit chopping to make sure it doesn't push into ExtendedID bits
    frameToPackage.len = 8;
    func_int32Value = int32_t1In;
    frameToPackage.buf[0] = func_uint8Value4X[3];
    frameToPackage.buf[1] = func_uint8Value4X[2];
    frameToPackage.buf[2] = func_uint8Value4X[1];
    frameToPackage.buf[3] = func_uint8Value4X[0];
    func_int32Value = int32_t2In;
    frameToPackage.buf[4] = func_uint8Value4X[3];
    frameToPackage.buf[5] = func_uint8Value4X[2];
    frameToPackage.buf[6] = func_uint8Value4X[1];
    frameToPackage.buf[7] = func_uint8Value4X[0];
    
    return frameToPackage;
}

CAN_message_t writeDouble4ByteDataCAN2Frame(uint16_t msgIDIn, throttlePoint point1In, throttlePoint point2In)
{
    // for double throttle point
    CAN_message_t frameToPackage;
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t func_uint32Value;             //unsigned 32 bit
        uint8_t func_uint8Value4X[4];
        uint16_t func_uint16Value2X[2];
        float func_floatValue = 0;
    };

    frameToPackage.flags.extended = 0;
    frameToPackage.flags.remote = 0;
    frameToPackage.id = msgIDIn;    //I should add bit chopping to make sure it doesn't push into ExtendedID bits
    frameToPackage.len = 8;
    func_uint16Value2X[0] = point1In.autoSequenceTimeValue/1000;
    func_uint16Value2X[1] = static_cast<uint16_t>(point1In.targetPcValue+0.5);
    frameToPackage.buf[0] = func_uint8Value4X[1];
    frameToPackage.buf[1] = func_uint8Value4X[0];
    frameToPackage.buf[2] = func_uint8Value4X[3];
    frameToPackage.buf[3] = func_uint8Value4X[2];
    func_uint16Value2X[0] = point2In.autoSequenceTimeValue/1000;
    func_uint16Value2X[1] = static_cast<uint16_t>(point2In.targetPcValue+0.5);
    frameToPackage.buf[4] = func_uint8Value4X[1];
    frameToPackage.buf[5] = func_uint8Value4X[0];
    frameToPackage.buf[6] = func_uint8Value4X[3];
    frameToPackage.buf[7] = func_uint8Value4X[2];
    
    return frameToPackage;
}

CAN_message_t writeDouble4ByteDataCAN2Frame(uint16_t msgIDIn, throttlePoint point1In)
{
    // for single throttle point
    CAN_message_t frameToPackage;
    union                       // union for storing bus bytes and pulling as desired value format
    {
        uint32_t func_uint32Value;             //unsigned 32 bit
        uint8_t func_uint8Value4X[4];
        uint16_t func_uint16Value2X[2];
        float func_floatValue = 0;
    };

    frameToPackage.flags.extended = 0;
    frameToPackage.flags.remote = 0;
    frameToPackage.id = msgIDIn;    //I should add bit chopping to make sure it doesn't push into ExtendedID bits
    frameToPackage.len = 4;
    func_uint16Value2X[0] = point1In.autoSequenceTimeValue/1000;
    func_uint16Value2X[1] = static_cast<uint16_t>(point1In.targetPcValue+0.5);
    frameToPackage.buf[0] = func_uint8Value4X[1];
    frameToPackage.buf[1] = func_uint8Value4X[0];
    frameToPackage.buf[2] = func_uint8Value4X[3];
    frameToPackage.buf[3] = func_uint8Value4X[2];
    
    return frameToPackage;
}


void FlexCan3Controller::generateHPObjectIDmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn)
{
    u_int16_t msgID;
    msgID = propulsionNodeIDIn+512+16;
    //for loop covers max 10 ALARA HP devices
    //Can skip limiting the size with a for loop when doing this by ALARA HP channels that are fixed at 10 max size on a given node
    //for (size_t i = 0; i < 10; i++)
    //{
        for (auto valve : valveArray)
        {
                if (valve->getValveNodeID() == propulsionNodeIDIn)
                {
                    nodeObjectIDReportStruct.objectIDByteArray[valve->getHPChannel()-1] = valve->getValveID();
                }
        }

        for (auto pyro : pyroArray)
        {
                if (pyro->getPyroNodeID() == propulsionNodeIDIn)
                {
                    nodeObjectIDReportStruct.objectIDByteArray[pyro->getHPChannel()-1] = pyro->getPyroID();
                }
        }
    //}
    writeObjectByteArray(nodeObjectIDReportStruct.objectIDByteArray, nodeObjectIDReportStruct.objectIDmsg, msgID);
}

void FlexCan3Controller::generateHPObjectStateReportmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn)
{
    u_int16_t msgID;
    msgID = propulsionNodeIDIn+512+32;
    //for loop covers max 10 ALARA HP devices
    //Can skip limiting the size with a for loop when doing this by ALARA HP channels that are fixed at 10 max size on a given node
    //for (size_t i = 0; i < 10; i++)
    //{
        for (auto valve : valveArray)
        {
                if (valve->getValveNodeID() == propulsionNodeIDIn)
                {
                    // add bit shifted electrical state later
                    nodeObjectStateReportStruct.objectIDByteArray[valve->getHPChannel()-1] = static_cast<uint8_t>(valve->getState());
                }
        }

        for (auto pyro : pyroArray)
        {
                if (pyro->getPyroNodeID() == propulsionNodeIDIn)
                {
                    // add bit shifted electrical state later
                    nodeObjectStateReportStruct.objectIDByteArray[pyro->getHPChannel()-1] = static_cast<uint8_t>(pyro->getState());
                }
        }
    //}
    writeObjectByteArray(nodeObjectStateReportStruct.objectIDByteArray, nodeObjectStateReportStruct.objectIDmsg, msgID);
}

void FlexCan3Controller::generateRawSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<SENSORBASE*, NUM_HPSENSORS>& HPsensorArray, const uint8_t& propulsionNodeIDIn)
{
    bool isFirstSample = true;
    ALARA_RawSensorReadmsg sensorReadStruct;
    
    
    //sensorReadStruct.timestapTolerance = 39*2; //number of micros precision approximately that this timestamp format will hold
    sensorReadStruct.timestampTolerance = 229; //number of micros precision approximately that this timestamp format will hold
    uint32_t currentIteratoinTimeStamp = 0;
    // grab up to 3 samples
    for (size_t i = 0; i < 3;)
    {
        //Serial.print(" :i of msg loop: ");
        //Serial.println(i);
        for (auto sensor : sensorArray)
        {
            if (sensor->getSensorNodeID() == propulsionNodeIDIn && sensor->getNewSensorValueCheckCAN()) // if on this node and a new value to send
            {
                //timestamp format is single seconds digits, measured down to micros precision
                currentIteratoinTimeStamp = ((sensor->getTimestampSeconds() % 10)*1000000) + sensor->getTimestampMicros();
                if (!isFirstSample)
                {
                    // if the next sample timestamp exceeds the tolerance range, break the for loop
                    if (currentIteratoinTimeStamp - sensorReadStruct.sensorTimestampMicros[0] >= sensorReadStruct.timestampTolerance)
                    {
                        break;
                    }
                }
                sensorReadStruct.sensorID[i] = sensor->getSensorID();
                //sensorReadStruct.sensorTimestampSeconds[i] = sensor->getTimestampSeconds();
                sensorReadStruct.sensorTimestampMicros[i] = currentIteratoinTimeStamp;

                // for Teensy ADC cast the raw int down to 16 bits
                if (sensor->getADCtype() == ADCType::TeensyMCUADC)
                {
                    sensorReadStruct.sensorRawValue[i] = static_cast<uint16_t>(sensor->getCurrentRawValue(true));
                }


                isFirstSample = false;
                
                i++;
        sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
                if (i == 3)
                {
                    break;
                }
            }
        
        //sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
        }
/*         for (auto sensor : HPsensorArray)
        {
            if (sensor->getSensorNodeID() == propulsionNodeIDIn && sensor->getNewSensorValueCheckCAN()) // if on this node and a new value to send
            {
                //timestamp format is single seconds digits, measured down to micros precision
                currentIteratoinTimeStamp = ((sensor->getTimestampSeconds() % 10)*1000000) + sensor->getTimestampMicros();
                if (!isFirstSample)
                {
                    // if the next sample timestamp exceeds the tolerance range, break the for loop
                    if (currentIteratoinTimeStamp - sensorReadStruct.sensorTimestampMicros[0] >= sensorReadStruct.timestampTolerance)
                    {
                        break;
                    }
                }
                sensorReadStruct.sensorID[i] = sensor->getSensorID();
                //sensorReadStruct.sensorTimestampSeconds[i] = sensor->getTimestampSeconds();
                sensorReadStruct.sensorTimestampMicros[i] = currentIteratoinTimeStamp;

                // for Teensy ADC cast the raw int down to 16 bits
                if (sensor->getADCtype() == ADCType::TeensyMCUADC)
                {
                    sensorReadStruct.sensorRawValue[i] = static_cast<uint16_t>(sensor->getCurrentRawValue(true));
                }


                isFirstSample = false;
                
                i++;
        sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
                if (i == 3)
                {
                    break;
                }
            }
        
        //sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
        }
 */
        break;  //if all sensors checked still break even if 3 samples have not been found

    }

        //Serial.print("do I get past i loop");
        //Serial.println(sensorReadStruct.numberSensors);
sensorReadStruct.packedSensorCAN2.flags.extended = 1;
sensorReadStruct.packedSensorCAN2.flags.remote = 0;
//sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0];
//sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + (sensorReadStruct.sensorTimestampMicros[0] << 11);
sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000) << 11);
//sensorReadStruct.packedSensorCAN2.id = (sensorReadStruct.sensorID[0], ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000)));
//sensorReadStruct.packedSensorCAN2.id = ((sensorReadStruct.sensorTimestampMicros[0]*0.0262143));

//below values are total frame bits including all overhead for this format using extended ID and the number of data bytes
if (sensorReadStruct.numberSensors == 1)
{
sensorReadStruct.packedSensorCAN2.len = 2;
sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorRawValue[0];
sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorRawValue[0] >> 8;
}
else if (sensorReadStruct.numberSensors == 2)
{
sensorReadStruct.packedSensorCAN2.len = 5;
sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorRawValue[0];
sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorRawValue[0] >> 8;
sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorRawValue[1];
sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorRawValue[1] >> 8;
}
else if (sensorReadStruct.numberSensors == 3)
{
sensorReadStruct.packedSensorCAN2.len = 8;
sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorRawValue[0];
sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorRawValue[0] >> 8;
sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorRawValue[1];
sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorRawValue[1] >> 8;
sensorReadStruct.packedSensorCAN2.buf[5] = sensorReadStruct.sensorID[2];
sensorReadStruct.packedSensorCAN2.buf[7] = sensorReadStruct.sensorRawValue[2];
sensorReadStruct.packedSensorCAN2.buf[6] = sensorReadStruct.sensorRawValue[2] >> 8;
}

sensorReadStruct.frameTotalBits = standardIDCAN2TotalBits[sensorReadStruct.packedSensorCAN2.flags.extended + 1][sensorReadStruct.packedSensorCAN2.len];


// write message to bus - for now just write immediately out of this function
// long term I think it's better for this to return the struct and have parent function use it to run through all the sensors ready to read each loop
if (sensorReadStruct.numberSensors >= 1)
{
/* Serial.println(" do I get to send raw CAN msg ");
        Serial.print(sensorReadStruct.numberSensors);
        Serial.print(" id: ");
        Serial.print(sensorReadStruct.packedSensorCAN2.id);
        Serial.print(" timestamp: ");
        Serial.print(sensorReadStruct.sensorTimestampMicros[0]);
        
      for (size_t i = 0; i < sensorReadStruct.packedSensorCAN2.len; i++)
      {
        Serial.print(" : ");
        Serial.print(sensorReadStruct.packedSensorCAN2.buf[i]);
        Serial.print(" : ");
      }
      Serial.println(); */
    

//CANbus.write(sensorReadStruct.packedSensorCAN2);
}

if (sensorReadStruct.numberSensors >= 1)
{
CANbus.write(sensorReadStruct.packedSensorCAN2);
}

}

//NOT CHANGED YET FROM RAW VERSION!!!! Needs to pull converted values at a given rate (if new available) and set the new converted false when reading just like with raw
//Use a one decimal place shifted version of the float as Int, will give up to ~6500.0 PSI max value on all the pressures.
bool FlexCan3Controller::generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<SENSORBASE*, NUM_HPSENSORS>& HPsensorArray, const uint8_t& propulsionNodeIDIn)
{
    //if (convertedValueUpdateTimer >= 0)
    //if (convertedValueUpdateTimer >= (1000/convertedSendRateHz))
    //{
        bool concersionChecksRemainingBool = true;  // use for return bool, set false at end of sensor array

        convertedValueUpdateTimer = 0;
        bool isFirstSample = true;
        ALARA_ConvertedSensorReadmsg sensorReadStruct;

        //auto sensorConvertedIt = sensorArray.begin();
        //std::array<SENSORBASE, NUM_SENSORS>::iterator sensorIt;
        //sensorIt = sensorArray.begin();

        sensorReadStruct.timestampTolerance = 1000000; //anything within 10 Hz window is good enough for packing converted values together
        uint32_t currentIteratoinTimeStamp = 0;
        // grab up to 3 samples
        for (size_t i = 0; i < 3;)
        {
            //Serial.print(" :i of msg loop: ");
            //Serial.println(i);
            //for (auto sensor : sensorArray)
            for (auto sensor : sensorArray)
            {
                if (sensor->getSensorNodeID() == propulsionNodeIDIn && sensor->getNewSensorConversionCheck()) // if on this node and a new value to send
                {
                    //timestamp format is single seconds digits, measured down to micros precision
                    currentIteratoinTimeStamp = ((sensor->getTimestampSeconds() % 10)*1000000) + sensor->getTimestampMicros();
                    if (!isFirstSample)
                    {
                        // if the next sample timestamp exceeds the tolerance range, break the for loop
                        if (currentIteratoinTimeStamp - sensorReadStruct.sensorTimestampMicros[0] >= sensorReadStruct.timestampTolerance)
                        {
                            break;
                        }
                    }
                    sensorReadStruct.sensorID[i] = (sensor->getSensorID())+1;
                    //sensorReadStruct.sensorTimestampSeconds[i] = sensor->getTimestampSeconds();
                    sensorReadStruct.sensorTimestampMicros[i] = currentIteratoinTimeStamp;

                    // for Teensy ADC cast the raw int down to 16 bits
                    if (sensor->getADCtype() == ADCType::TeensyMCUADC)
                    {
                        sensorReadStruct.sensorConvertedValue[i] = static_cast<uint16_t>(sensor->getCurrentConvertedValue(true)*10);
                    }


                    isFirstSample = false;
                    
                    i++;
            sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
                    if (i == 3)
                    {
                        break;
                    }
                }
            
            
            //if (sensorArray.end() == sensor)
            //{
                /* code */
            //}
            }
            for (auto sensor : HPsensorArray)
            {
                if (sensor->getSensorNodeID() == propulsionNodeIDIn && sensor->getNewSensorConversionCheck()) // if on this node and a new value to send
                {
                    //timestamp format is single seconds digits, measured down to micros precision
                    currentIteratoinTimeStamp = ((sensor->getTimestampSeconds() % 10)*1000000) + sensor->getTimestampMicros();
                    if (!isFirstSample)
                    {
                        // if the next sample timestamp exceeds the tolerance range, break the for loop
                        if (currentIteratoinTimeStamp - sensorReadStruct.sensorTimestampMicros[0] >= sensorReadStruct.timestampTolerance)
                        {
                            break;
                        }
                    }
                    sensorReadStruct.sensorID[i] = (sensor->getSensorID());
                    //sensorReadStruct.sensorTimestampSeconds[i] = sensor->getTimestampSeconds();
                    sensorReadStruct.sensorTimestampMicros[i] = currentIteratoinTimeStamp;

                    // for Teensy ADC cast the raw int down to 16 bits
                    if (sensor->getADCtype() == ADCType::TeensyMCUADC)
                    {
                        sensorReadStruct.sensorConvertedValue[i] = static_cast<uint16_t>(sensor->getCurrentConvertedValue(true)*10);
                    }


                    isFirstSample = false;
                    
                    i++;
            sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
                    if (i == 3)
                    {
                        break;
                    }
                }
            
            
            //if (sensorArray.end() == sensor)
            //{
                /* code */
            //}
            }
            break;  //if all sensors checked still break even if 3 samples have not been found

        }

            //Serial.print("do I get past i loop");
            //Serial.println(sensorReadStruct.numberSensors);
    sensorReadStruct.packedSensorCAN2.flags.extended = 1;
    sensorReadStruct.packedSensorCAN2.flags.remote = 0;
    sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000) << 11);

    //below values are total frame bits including all overhead for this format using extended ID and the number of data bytes
    if (sensorReadStruct.numberSensors == 1)
    {
    sensorReadStruct.packedSensorCAN2.len = 2;
    sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorConvertedValue[0];
    sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorConvertedValue[0] >> 8;
    }
    else if (sensorReadStruct.numberSensors == 2)
    {
    sensorReadStruct.packedSensorCAN2.len = 5;
    sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorConvertedValue[0];
    sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorConvertedValue[0] >> 8;
    sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
    sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorConvertedValue[1];
    sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorConvertedValue[1] >> 8;
    }
    else if (sensorReadStruct.numberSensors == 3)
    {
    sensorReadStruct.packedSensorCAN2.len = 8;
    sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorConvertedValue[0];
    sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorConvertedValue[0] >> 8;
    sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
    sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorConvertedValue[1];
    sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorConvertedValue[1] >> 8;
    sensorReadStruct.packedSensorCAN2.buf[5] = sensorReadStruct.sensorID[2];
    sensorReadStruct.packedSensorCAN2.buf[7] = sensorReadStruct.sensorConvertedValue[2];
    sensorReadStruct.packedSensorCAN2.buf[6] = sensorReadStruct.sensorConvertedValue[2] >> 8;
    }

    sensorReadStruct.frameTotalBits = standardIDCAN2TotalBits[sensorReadStruct.packedSensorCAN2.flags.extended + 1][sensorReadStruct.packedSensorCAN2.len];


    // write message to bus - for now just write immediately out of this function
    // long term I think it's better for this to return the struct and have parent function use it to run through all the sensors ready to read each loop
    if (sensorReadStruct.numberSensors >= 1)
    {
/*     Serial.println(" do I get to send Converted CAN msg ");
            Serial.print(sensorReadStruct.numberSensors);
            Serial.print(" id: ");
            Serial.print(sensorReadStruct.packedSensorCAN2.id);
            Serial.print(" timestamp: ");
            Serial.print(sensorReadStruct.sensorTimestampMicros[0]);
            
        for (size_t i = 0; i < sensorReadStruct.packedSensorCAN2.len; i++)
        {
            Serial.print(" : ");
            Serial.print(sensorReadStruct.packedSensorCAN2.buf[i]);
            Serial.print(" : ");
        }
        Serial.println(); */
        

    //CANbus.write(sensorReadStruct.packedSensorCAN2);
    }

    if (sensorReadStruct.numberSensors >= 1)
    {
    CANbus.write(sensorReadStruct.packedSensorCAN2);
    }
    //}
    return isFirstSample; // for later use when I get iterators working inside here
}

void FlexCan3Controller::generateTankControllermsgs(FlexCAN& CANbus, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const uint8_t& propulsionNodeIDIn)
{
/*     fuelTankPressControllerReportsStruct.tankControllerCAN2Frames.reserve(8);
    loxTankPressControllerReportsStruct.tankControllerCAN2Frames.reserve(8);
    fuelTankPressControllerReportsStruct.tankControllerCAN2Frames.resize(8);
    loxTankPressControllerReportsStruct.tankControllerCAN2Frames.resize(8);
 */    
    for (auto tankPressController : tankPressControllerArray)
    {
        if (tankPressController->getIsBang())
        {
            tankPressControllerReportsStruct.controllerID = tankPressController->getControllerID();
            tankPressControllerReportsStruct.controllerStateReportID = (tankPressController->getControllerID()*100) + 1000;
            
            // Controller State Report and other quasistatic info to send low rate
            //if (tankPressControllerReportsStruct.quasistaticSendBool || externalStateChange)
            if (tankPressController->getControllerConfigUpdate())
            {
            tankPressControllerReportsStruct.controllerStateReportCanFrame.id = tankPressControllerReportsStruct.controllerStateReportID;
            tankPressControllerReportsStruct.controllerStateReportCanFrame.flags.extended = 0;
            tankPressControllerReportsStruct.controllerStateReportCanFrame.flags.remote = 0;
            tankPressControllerReportsStruct.controllerStateReportCanFrame.buf[0] = static_cast<uint8_t>(tankPressController->getState());
            //CANbus.write(tankPressControllerReportsStruct.controllerStateReportCanFrame);
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 2,tankPressController->getKp(),tankPressController->getKi()));
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 4,tankPressController->getKd(),tankPressController->getControllerThreshold()));
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 14,tankPressController->getValveMinEnergizeTime(),tankPressController->getValveMinDeEnergizeTime()));
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 16,tankPressController->getVentFailsafePressure()));
            tankPressController->setControllerConfigUpdate(false);
            //tankPressControllerReportsStruct.quasistaticSendBool = false;
            }

            if (tankPressController->getControllerUpdate())
            {
                // Messages to send every controller update
                //
                CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 6,tankPressController->getPfunc(),tankPressController->getP_p()));
                //
                CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 8,tankPressController->getIfunc(),tankPressController->getP_i()));
                //
                CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 10,tankPressController->getDfunc(),tankPressController->getP_d()));
                //
                CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 12,tankPressController->getTargetValue(),tankPressController->getPIDoutput()));
                //reset controller update bool after grabbing all the data
                tankPressController->setControllerUpdate(false);
            

            }
            
        }
        // If not bang . . .
        else
        {
            tankPressControllerReportsStruct.controllerID = tankPressController->getControllerID();
            tankPressControllerReportsStruct.controllerStateReportID = (tankPressController->getControllerID()*100) + 1000;
            
            // Controller State Report and other quasistatic info to send low rate
            //if (tankPressControllerReportsStruct.quasistaticSendBool || externalStateChange)
            if (tankPressController->getControllerConfigUpdate())
            {
            tankPressControllerReportsStruct.controllerStateReportCanFrame.id = tankPressControllerReportsStruct.controllerStateReportID;
            tankPressControllerReportsStruct.controllerStateReportCanFrame.flags.extended = 0;
            tankPressControllerReportsStruct.controllerStateReportCanFrame.flags.remote = 0;
            tankPressControllerReportsStruct.controllerStateReportCanFrame.buf[0] = static_cast<uint8_t>(tankPressController->getState());
            //CANbus.write(tankPressControllerReportsStruct.controllerStateReportCanFrame);
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 2,tankPressController->getKp(),tankPressController->getKi()));
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 4,tankPressController->getKd(),tankPressController->getControllerThreshold()));
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 14,tankPressController->getValveMinEnergizeTime(),tankPressController->getValveMinDeEnergizeTime()));
            CANbus.write(writeDouble4ByteDataCAN2Frame(tankPressControllerReportsStruct.controllerStateReportID + 16,tankPressController->getVentFailsafePressure()));
            tankPressController->setControllerConfigUpdate(false);
            //tankPressControllerReportsStruct.quasistaticSendBool = false;
            }
        }
        
    }
}

void FlexCan3Controller::generateEngineControllermsgs(FlexCAN& CANbus, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, const uint8_t& propulsionNodeIDIn)
{
    for (auto engine : engineControllerArray)
    {
        // Throttle program
        uint16_t controllerID = 1000 + (engine->getControllerID()*100);
        throttlePoint point1;
        throttlePoint point2;

        bool point1bool = false;
        bool point2bool = false;
        if (engine->getControllerConfigUpdate())
        {
            for (auto i = engine->throttleProgram.begin(); i != engine->throttleProgram.end();)
            {
                if (!point1bool)
                {
                    //Serial.print(" does point 1 run: ");
                    point1.autoSequenceTimeValue = i->autoSequenceTimeValue;
                    point1.targetPcValue = i->targetPcValue;
                    point1bool = true;
                    i++;
                }
                else if (!point2bool)
                {
                    //Serial.print(" does point 2 run: ");
                    point2.autoSequenceTimeValue = i->autoSequenceTimeValue;
                    point2.targetPcValue = i->targetPcValue;
                    point2bool = true;
                    i++;
                }
                if (point1bool && point2bool)
                {
                //Serial.println(" does 2 point msg send: ");
                CANbus.write(writeDouble4ByteDataCAN2Frame((controllerID + 6), point1, point2));
                point1bool = false;
                point2bool = false;
                }
                else if (i == (engine->throttleProgram.end()) && point1bool)
                {
                //Serial.println(" does 1 point msg send: ");
                CANbus.write(writeDouble4ByteDataCAN2Frame((controllerID + 6), point1));
                point1bool = false;
                point2bool = false;
                }
                else;
            }
            CANbus.write(writeDouble4ByteDataCAN2Frame((controllerID + 2), engine->getFuelMVAutosequenceActuation(),engine->getLoxMVAutosequenceActuation()));
            CANbus.write(writeDouble4ByteDataCAN2Frame((controllerID + 4), engine->getIgniter1Actuation(),engine->getIgniter2Actuation()));
            //reset controller update once data sent
            engine->setControllerConfigUpdate(false);
        }
    }
}

void FlexCan3Controller::generateAutoSequenceUpdatemsg(FlexCAN& CANbus, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const uint8_t& propulsionNodeIDIn)
{
    
    //if (AutoSequenceReportTimer >= 100)
    //{
    // build message
        static CAN_message_t msgOut;
        msgOut.flags.extended = 0;
        msgOut.flags.remote = 0;
        //change ID format to be better and match my updated plan
        for(auto autoSequence : autoSequenceArray)
        if (autoSequence->getHostNodeID() == propulsionNodeIDIn)
        {
            {
                msgOut.id = 1000 + (autoSequence->getAutoSequenceID()*100);  //
                msgOut.len = 8;
                int64_t autosequenceTimer = autoSequence->getCurrentCountdown();
                //uint8_t autosequenceTimerStateEnumToInt = static_cast<uint8_t>(autoSequence->getAutoSequenceState());

/*                 Serial.print("Autosequence: State : ");
                Serial.print(autosequenceTimerStateEnumToInt);
                Serial.print(" Timer : ");
                Serial.print(autosequenceTimer);
                Serial.println(); */

                //msgOut.buf[0] = autosequenceTimerStateEnumToInt;
                //is this bit shifting logically backwards???
/*                 msgOut.buf[0] = autosequenceTimer;
                msgOut.buf[1] = (autosequenceTimer >> 8);
                msgOut.buf[2] = (autosequenceTimer >> 16);
                msgOut.buf[3] = (autosequenceTimer >> 24);
                msgOut.buf[4] = (autosequenceTimer >> 32);
                msgOut.buf[5] = (autosequenceTimer >> 40);
                msgOut.buf[6] = (autosequenceTimer >> 48);
                msgOut.buf[7] = (autosequenceTimer >> 56);
 */

                msgOut.buf[0] = (autosequenceTimer >> 56);
                msgOut.buf[1] = (autosequenceTimer >> 48);
                msgOut.buf[2] = (autosequenceTimer >> 40);
                msgOut.buf[3] = (autosequenceTimer >> 32);
                msgOut.buf[4] = (autosequenceTimer >> 24);
                msgOut.buf[5] = (autosequenceTimer >> 16);
                msgOut.buf[6] = (autosequenceTimer >> 8);
                msgOut.buf[7] = autosequenceTimer;

                // write message to bus
                CANbus.write(msgOut);
            }    
            {
                // add write error handling here, for now it does nothing
            }
            //AutoSequenceReportTimer = 0;
        }
    //}
}

void FlexCan3Controller::generateFluidSimmsgs(FlexCAN& CANbus, FluidSystemSimulation& fluidSim, const uint8_t& propulsionNodeIDIn)
{
    // Get all the things for Fluid Sim settings in here
}

void FlexCan3Controller::generatePropNodeStateReport(FlexCAN& CANbus,  VehicleState& currentState, MissionState& currentMissionState, Command& currentCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const uint8_t& propulsionNodeIDIn)
{
    CAN_message_t stateReport;

    stateReport.len = 8; //always a full frame this format
    stateReport.flags.extended = 0;
    stateReport.flags.remote = 0;
    stateReport.id = propulsionNodeIDIn + 512;   //standard ID plus pack first two bytes into back of extendedID field
    
    stateReport.buf[0] = static_cast<uint8_t>(currentState);
    stateReport.buf[1] = static_cast<uint8_t>(currentMissionState);
    stateReport.buf[2] = static_cast<uint8_t>(currentCommand);
    stateReport.buf[3] = static_cast<uint8_t>(autoSequenceArray.at(0)->getAutoSequenceState());
    stateReport.buf[4] = static_cast<uint8_t>(engineControllerArray.at(0)->getState());
    stateReport.buf[5] = static_cast<uint8_t>(tankPressControllerArray.at(0)->getState());
    stateReport.buf[6] = static_cast<uint8_t>(tankPressControllerArray.at(1)->getState());
    stateReport.buf[7] = static_cast<uint8_t>(tankPressControllerArray.at(2)->getState());

    //writeNodeStateReportByteArray(stateReportArray, stateReport, propulsionNodeIDIn);
    CANbus.write(stateReport);
}

void FlexCan3Controller::controllerTasks(FlexCAN& CANbus, VehicleState& currentState, MissionState& currentMissionState, Command& currentCommand, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<SENSORBASE*, NUM_HPSENSORS>& HPsensorArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, FluidSystemSimulation& fluidSim, const uint8_t& propulsionNodeIDIn)
{
    //call this every loop of main program
    //call all the types of messages inside this function and execute as needed
    if (propNodeStateReportTimer >= propNodeStateReportRateMillis)
    {
    generatePropNodeStateReport(CANbus, currentState, currentMissionState, currentCommand, autoSequenceArray, engineControllerArray, tankPressControllerArray, propulsionNodeIDIn);
    propNodeStateReportTimer = 0;
    }

    if (!objectIDmsgs)  //generate the message once
    {
    generateHPObjectIDmsgs(CANbus, valveArray, pyroArray, propulsionNodeIDIn);
    objectIDmsgs = true;
    }
    if (highPowerObjectIDmsgTimer >= highPowerObjectIDRateMillis) //send low rate ping of what objects are on this node HP channels
    {
    //
    CANbus.write(nodeObjectIDReportStruct.objectIDmsg);
    highPowerObjectIDmsgTimer = 0; 
    }
    
    if (highPowerStatemsgTimer >= highPowerStateReportRateMillis || externalStateChange)
    {
        // make and send high power state report
    generateHPObjectStateReportmsgs(CANbus, valveArray, pyroArray, propulsionNodeIDIn);
    CANbus.write(nodeObjectStateReportStruct.objectIDmsg);    
    highPowerStatemsgTimer = 0;
    }
    
    if (convertedValueUpdateTimer >= (convertedSendRateMillis))
    {
    //auto sensorConvertedIt = sensorArray.begin();
        // Lazy way to make sure I send all the possible queued up conversions
        generateConvertedSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
        generateConvertedSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
        generateConvertedSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
        generateConvertedSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
        generateConvertedSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
        convertedValueUpdateTimer = 0;
    }

    //set this up to run until raw messages all cleared each loop
    generateRawSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
    generateRawSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);
    generateRawSensormsgs(Can0, sensorArray, HPsensorArray, propulsionNodeIDIn);

    if (AutoSequenceReportTimer >= 200)
    {
        generateAutoSequenceUpdatemsg(Can0, autoSequenceArray, propulsionNodeIDIn);
        AutoSequenceReportTimer = 0;
    }


    generateTankControllermsgs(Can0,tankPressControllerArray,propulsionNodeIDIn);
    generateEngineControllermsgs(Can0,engineControllerArray,propulsionNodeIDIn);
    generateFluidSimmsgs(Can0, fluidSim, propulsionNodeIDIn);
    //reset external state change bool
    externalStateChange = false;
}