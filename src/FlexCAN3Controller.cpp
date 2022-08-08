#include "FlexCAN3Controller.h"

// CAN2 bit overhead precalculated values, row 0 is number of data bytes, row 1 is bits with standard ID only, row 2 is bits with extended ID 
// Values are likely WRONG to some degree, my calculatons don't match wiki stated values but good enough for rough bus load estimation
uint8_t standardIDCAN2TotalBits[3][9] = {{0,1,2,3,4,5,6,7,8},{52,62,72,82,92,102,112,122,132},{77,87,97,107,117,127,137,147,157}};
const uint32_t max18bitvalue = 262143;  //preset as constant to not calculate it

void FlexCan3Controller::generateObjectIDmsgs(FlexCAN& CANbus, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const uint8_t& propulsionNodeIDIn)
{
    //for loop covers max 10 ALARA HP devices
    for (size_t i = 0; i < 10; i++)
    {
        // split ALARA HP devices across possibly 2 CAN2 frames
        for (size_t j = 0; j < 8; j++)
        {
            for (auto valve : valveArray)
            {
                    if (valve->getValveNodeID() == propulsionNodeIDIn)
                    {
                        nodeReportStruct.objectIDmsg1.buf[j] = valve->getValveID();
                    }
            }

            for (auto pyro : pyroArray)
            {
                    if (pyro->getPyroNodeID() == propulsionNodeIDIn)
                    {
                        nodeReportStruct.objectIDmsg1.buf[j] = pyro->getPyroID();
                    }
            }
        }
    }
}

void FlexCan3Controller::generateRawSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn)
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
        break;  //if all sensors checked still break even if 3 samples have not been found

    }

        //Serial.print("do I get past i loop");
        //Serial.println(sensorReadStruct.numberSensors);
//sensorReadStruct.packedSensorCAN2.flags.extended = 1;
//sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0];
//sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + (sensorReadStruct.sensorTimestampMicros[0] << 11);
sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000) << 11);
//sensorReadStruct.packedSensorCAN2.id = ((sensorReadStruct.sensorTimestampMicros[0]*0.0262143));

//below values are total frame bits including all overhead for this format using extended ID and the number of data bytes
if (sensorReadStruct.numberSensors == 1)
{
sensorReadStruct.packedSensorCAN2.len = 2;
sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorRawValue[0];
sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorRawValue[0] >> 8;
}
else if (sensorReadStruct.numberSensors == 2)
{
sensorReadStruct.packedSensorCAN2.len = 5;
sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorRawValue[0];
sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorRawValue[0] >> 8;
sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorRawValue[1];
sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorRawValue[1] >> 8;
}
else if (sensorReadStruct.numberSensors == 3)
{
sensorReadStruct.packedSensorCAN2.len = 8;
sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorRawValue[0];
sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorRawValue[0] >> 8;
sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorRawValue[1];
sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorRawValue[1] >> 8;
sensorReadStruct.packedSensorCAN2.buf[5] = sensorReadStruct.sensorID[2];
sensorReadStruct.packedSensorCAN2.buf[6] = sensorReadStruct.sensorRawValue[2];
sensorReadStruct.packedSensorCAN2.buf[7] = sensorReadStruct.sensorRawValue[2] >> 8;
}

sensorReadStruct.frameTotalBits = standardIDCAN2TotalBits[sensorReadStruct.packedSensorCAN2.flags.extended + 1][sensorReadStruct.packedSensorCAN2.len];


// write message to bus - for now just write immediately out of this function
// long term I think it's better for this to return the struct and have parent function use it to run through all the sensors ready to read each loop
if (sensorReadStruct.numberSensors >= 1)
{
Serial.println(" do I get to send raw CAN msg ");
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
      Serial.println();
    

//CANbus.write(sensorReadStruct.packedSensorCAN2);
}

if (sensorReadStruct.numberSensors >= 1)
{
CANbus.write(sensorReadStruct.packedSensorCAN2);
}

}

//NOT CHANGED YET FROM RAW VERSION!!!! Needs to pull converted values at a given rate (if new available) and set the new converted false when reading just like with raw
//Use a one decimal place shifted version of the float as Int, will give up to ~6500.0 PSI max value on all the pressures.
void FlexCan3Controller::generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn, uint32_t convertedSendRateHz)
{
    //if (convertedValueUpdateTimer >= 0)
    if (convertedValueUpdateTimer >= (1000/convertedSendRateHz))
    {
        convertedValueUpdateTimer = 0;
        bool isFirstSample = true;
        ALARA_ConvertedSensorReadmsg sensorReadStruct;
        
        
        sensorReadStruct.timestampTolerance = 1000000; //anything within 10 Hz window is good enough for packing converted values together
        uint32_t currentIteratoinTimeStamp = 0;
        // grab up to 3 samples
        for (size_t i = 0; i < 3;)
        {
            //Serial.print(" :i of msg loop: ");
            //Serial.println(i);
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
                    sensorReadStruct.sensorID[i] = sensor->getSensorID();
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
            
            //sensorReadStruct.numberSensors = i; //I think, might need to be just i and I'm dumb
            }
            break;  //if all sensors checked still break even if 3 samples have not been found

        }

            //Serial.print("do I get past i loop");
            //Serial.println(sensorReadStruct.numberSensors);
    //sensorReadStruct.packedSensorCAN2.flags.extended = 1;
    sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000) << 11);

    //below values are total frame bits including all overhead for this format using extended ID and the number of data bytes
    if (sensorReadStruct.numberSensors == 1)
    {
    sensorReadStruct.packedSensorCAN2.len = 2;
    sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorConvertedValue[0];
    sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorConvertedValue[0] >> 8;
    }
    else if (sensorReadStruct.numberSensors == 2)
    {
    sensorReadStruct.packedSensorCAN2.len = 5;
    sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorConvertedValue[0];
    sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorConvertedValue[0] >> 8;
    sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
    sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorConvertedValue[1];
    sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorConvertedValue[1] >> 8;
    }
    else if (sensorReadStruct.numberSensors == 3)
    {
    sensorReadStruct.packedSensorCAN2.len = 8;
    sensorReadStruct.packedSensorCAN2.buf[0] = sensorReadStruct.sensorConvertedValue[0];
    sensorReadStruct.packedSensorCAN2.buf[1] = sensorReadStruct.sensorConvertedValue[0] >> 8;
    sensorReadStruct.packedSensorCAN2.buf[2] = sensorReadStruct.sensorID[1];
    sensorReadStruct.packedSensorCAN2.buf[3] = sensorReadStruct.sensorConvertedValue[1];
    sensorReadStruct.packedSensorCAN2.buf[4] = sensorReadStruct.sensorConvertedValue[1] >> 8;
    sensorReadStruct.packedSensorCAN2.buf[5] = sensorReadStruct.sensorID[2];
    sensorReadStruct.packedSensorCAN2.buf[6] = sensorReadStruct.sensorConvertedValue[2];
    sensorReadStruct.packedSensorCAN2.buf[7] = sensorReadStruct.sensorConvertedValue[2] >> 8;
    }

    sensorReadStruct.frameTotalBits = standardIDCAN2TotalBits[sensorReadStruct.packedSensorCAN2.flags.extended + 1][sensorReadStruct.packedSensorCAN2.len];


    // write message to bus - for now just write immediately out of this function
    // long term I think it's better for this to return the struct and have parent function use it to run through all the sensors ready to read each loop
    if (sensorReadStruct.numberSensors >= 1)
    {
    Serial.println(" do I get to send Converted CAN msg ");
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
        Serial.println();
        

    //CANbus.write(sensorReadStruct.packedSensorCAN2);
    }

    if (sensorReadStruct.numberSensors >= 1)
    {
    CANbus.write(sensorReadStruct.packedSensorCAN2);
    }
    }
}