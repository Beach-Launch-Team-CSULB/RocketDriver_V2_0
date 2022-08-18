#include "FlexCAN3Controller.h"

// CAN2 bit overhead precalculated values, row 0 is number of data bytes, row 1 is bits with standard ID only, row 2 is bits with extended ID 
// Values are likely WRONG to some degree, my calculatons don't match wiki stated values but good enough for rough bus load estimation
uint8_t standardIDCAN2TotalBits[3][9] = {{0,1,2,3,4,5,6,7,8},{52,62,72,82,92,102,112,122,132},{77,87,97,107,117,127,137,147,157}};
const uint32_t max18bitvalue = 262143;  //preset as constant to not calculate it

void FlexCan3Controller::writeObjectByteArray(uint8_t byteArray[10], CAN_message_t& msgIn, uint16_t IDA)
{
    msgIn.len = 8; //always a full frame this format
    msgIn.ext = 1;
    msgIn.id = IDA + (byteArray[0] << 13) + (byteArray[1] << 21);   //standard ID plus pack first two bytes into back of extendedID field
    for (size_t i = 0; i < 8; i++)
    {
        msgIn.buf[i] = byteArray[i+2];  //pack the back 8 elements of byte array into normal CAN2 bytes
    }
    //return msgIn;
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

    frameToPackage.id = msgIDIn;
    func_floatValue = float1In;
    frameToPackage.buf[0] = func_uint8Value4X[0];
    frameToPackage.buf[1] = func_uint8Value4X[1];
    frameToPackage.buf[2] = func_uint8Value4X[2];
    frameToPackage.buf[3] = func_uint8Value4X[3];
    func_floatValue = float2In;
    frameToPackage.buf[4] = func_uint8Value4X[0];
    frameToPackage.buf[5] = func_uint8Value4X[1];
    frameToPackage.buf[6] = func_uint8Value4X[2];
    frameToPackage.buf[7] = func_uint8Value4X[3];
    
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

    frameToPackage.id = msgIDIn;
    func_uint32Value = uint32_t1In;
    frameToPackage.buf[0] = func_uint8Value4X[0];
    frameToPackage.buf[1] = func_uint8Value4X[1];
    frameToPackage.buf[2] = func_uint8Value4X[2];
    frameToPackage.buf[3] = func_uint8Value4X[3];
    func_uint32Value = uint32_t2In;
    frameToPackage.buf[4] = func_uint8Value4X[0];
    frameToPackage.buf[5] = func_uint8Value4X[1];
    frameToPackage.buf[6] = func_uint8Value4X[2];
    frameToPackage.buf[7] = func_uint8Value4X[3];
    
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
sensorReadStruct.packedSensorCAN2.flags.extended = 1;
//sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0];
//sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + (sensorReadStruct.sensorTimestampMicros[0] << 11);
sensorReadStruct.packedSensorCAN2.id = sensorReadStruct.sensorID[0] + ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000) << 11);
//sensorReadStruct.packedSensorCAN2.id = (sensorReadStruct.sensorID[0], ((uint64_t(sensorReadStruct.sensorTimestampMicros[0])*uint64_t(max18bitvalue)/10000000)));
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
void FlexCan3Controller::generateConvertedSensormsgs(FlexCAN& CANbus, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn)
{
    //if (convertedValueUpdateTimer >= 0)
    //if (convertedValueUpdateTimer >= (1000/convertedSendRateHz))
    //{
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
    sensorReadStruct.packedSensorCAN2.ext = 1;
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
}

void FlexCan3Controller::generateTankControllermsgs(FlexCAN& CANbus, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const uint8_t& propulsionNodeIDIn)
{
    uint16_t controllerStateReportID;
    for (auto tankPressController : tankPressControllerArray)
    {
        if (tankPressController->getIsBang())
        {
            if (tankPressController->getControllerUpdate())
            {
                if (tankPressController->getControllerID() == 3) // Lox tank controller
                {
                controllerStateReportID = (tankPressController->getControllerID()*100) + 1000;
                // Controller State Report
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[0].id = controllerStateReportID;
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[1].buf[0] = static_cast<uint8_t>(tankPressController->getState());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[1] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 2,tankPressController->getKp(),tankPressController->getKi());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[2] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 4,tankPressController->getKd(),tankPressController->getControllerThreshold());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[3] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 6,tankPressController->getPfunc(),tankPressController->getP_p());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[4] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 8,tankPressController->getIfunc(),tankPressController->getP_i());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[5] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 10,tankPressController->getDfunc(),tankPressController->getP_d());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[6] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 12,tankPressController->getTargetValue(),tankPressController->getPIDoutput());
                //
                loxTankPressControllerReportsStruct.tankControllerCAN2Frames[7] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 14,tankPressController->getValveMinEnergizeTime(),tankPressController->getValveMinDeEnergizeTime());
                //reset controller update bool after grabbing all the data
                tankPressController->setControllerUpdate(false);
                }

                if (tankPressController->getControllerID() == 4) // Fuel tank controller
                {
                controllerStateReportID = (tankPressController->getControllerID()*100) + 1000;
                // Controller State Report
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[0].id = controllerStateReportID;
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[1].buf[0] = static_cast<uint8_t>(tankPressController->getState());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[1] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 2,tankPressController->getKp(),tankPressController->getKi());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[2] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 4,tankPressController->getKd(),tankPressController->getControllerThreshold());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[3] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 6,tankPressController->getPfunc(),tankPressController->getP_p());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[4] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 8,tankPressController->getIfunc(),tankPressController->getP_i());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[5] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 10,tankPressController->getDfunc(),tankPressController->getP_d());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[6] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 12,tankPressController->getTargetValue(),tankPressController->getPIDoutput());
                //
                fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[7] = writeDouble4ByteDataCAN2Frame(controllerStateReportID + 14,tankPressController->getValveMinEnergizeTime(),tankPressController->getValveMinDeEnergizeTime());
                //reset controller update bool after grabbing all the data
                tankPressController->setControllerUpdate(false);
                }
            }
            
        }
        
    }
    for (size_t i = 0; i < 8; i++)
    {
        // messages 3 through 6 send every update
        if (i >= 3 && i <= 7)
        {
            CANbus.write(loxTankPressControllerReportsStruct.tankControllerCAN2Frames[i]);
        }
        // messages 1, 2, 7 send only at low rate timer frequency
        else if (loxTankPressControllerReportsStruct.quasistaticUpdateTimer >= loxTankPressControllerReportsStruct.quasistaticSendTime)
        {
            CANbus.write(loxTankPressControllerReportsStruct.tankControllerCAN2Frames[i]);
            //reset send timer
            loxTankPressControllerReportsStruct.quasistaticUpdateTimer = 0;
        }

    }
    for (size_t i = 0; i < 8; i++)
    {
        // messages 3 through 6 send every update
        if (i >= 3 && i <= 7)
        {
            CANbus.write(fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[i]);
        }
        // messages 1, 2, 7 send only at low rate timer frequency
        else if (fuelTankPressControllerReportsStruct.quasistaticUpdateTimer >= fuelTankPressControllerReportsStruct.quasistaticSendTime)
        {
            CANbus.write(fuelTankPressControllerReportsStruct.tankControllerCAN2Frames[i]);
            //reset send timer
            fuelTankPressControllerReportsStruct.quasistaticUpdateTimer = 0;
        }

    }
    
}



void FlexCan3Controller::controllerTasks(FlexCAN& CANbus, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn)
{
    //call this every loop of main program
    //call all the types of messages inside this function and execute as needed
    if (!objectIDmsgs)  //generate the message once
    {
    generateHPObjectIDmsgs(CANbus, valveArray, pyroArray, propulsionNodeIDIn);
    objectIDmsgs = true;
    }
    if (highPowerObjectIDmsgTimer >= (1000/(highPowerObjectIDRateHz/highPowerObjectIDRateHzDenominator))) //send low rate ping of what objects are on this node HP channels
    {
    //CANbus.write(nodeObjectIDReportStruct.objectIDmsg);
    highPowerObjectIDmsgTimer = 0;
    
/*     Serial.println(" do I get to send ObjectID CAN msg ");
            Serial.print(" id: ");
            Serial.print(nodeObjectIDReportStruct.objectIDmsg.id);
            //Serial.print(" timestamp: ");
            //Serial.print(sensorReadStruct.sensorTimestampMicros[0]);
            
        for (size_t i = 0; i < nodeObjectIDReportStruct.objectIDmsg.len; i++)
        {
            Serial.print(" : ");
            Serial.print(nodeObjectIDReportStruct.objectIDmsg.buf[i]);
            Serial.print(" : ");
        }
        Serial.println();   */  
    }
    
    if (highPowerStatemsgTimer >= (1000/highPowerStateReportRateHz))
    {
        // make and send high power state report
    generateHPObjectStateReportmsgs(CANbus, valveArray, pyroArray, propulsionNodeIDIn);
    //nodeObjectStateReportStruct.objectIDmsg.flags.extended = 1;
    CANbus.write(nodeObjectStateReportStruct.objectIDmsg);    
        highPowerStatemsgTimer = 0;

    Serial.println(" do I get to send Object State CAN msg ");
            Serial.print(" id: ");
            Serial.print(nodeObjectStateReportStruct.objectIDmsg.id);
            //Serial.print(" timestamp: ");
            //Serial.print(sensorReadStruct.sensorTimestampMicros[0]);
            
        for (size_t i = 0; i < nodeObjectStateReportStruct.objectIDmsg.len; i++)
        {
            Serial.print(" : ");
            Serial.print(nodeObjectStateReportStruct.objectIDmsg.buf[i]);
            Serial.print(" : ");
        }
        Serial.println();    
    }
    
    if (convertedValueUpdateTimer >= (1000/convertedSendRateHz))
    {
        //generateConvertedSensormsgs(Can0, sensorArray, propulsionNodeIDIn);
        convertedValueUpdateTimer = 0;
    
    
    }

    //set this up to run until raw messages all cleared each loop
    generateRawSensormsgs(Can0, sensorArray, propulsionNodeIDIn);

    generateTankControllermsgs(Can0,tankPressControllerArray,propulsionNodeIDIn);
}