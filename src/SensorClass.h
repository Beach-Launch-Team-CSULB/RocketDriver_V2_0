#ifndef SENSORCLASS_H
#define SENSORSCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "SensorStates.h"

// Base sensor class to hold the various derived classes, should be purely virtual functions


class SENSORBASE
{
  public:
    virtual void begin();                     //
    virtual void read();              // updates currentRawValue with current reading, using an activated ADC object
    virtual void stateOperations();

    // Access functions defined in place
    virtual uint32_t getSensorID();
    virtual uint32_t getSensorNodeID();
    virtual uint32_t getADCinput();
    virtual uint32_t getCurrentSampleRate();
    virtual uint32_t getCurrentRawValue();
    virtual float getCurrentConvertedValue();
    virtual uint16_t getCANTimestamp();
    virtual uint32_t getTimestampSeconds();
    virtual uint32_t getTimestampMicros();
    //virtual uint8_t getCurrentRollingArrayPosition(){return currentRollingArrayPosition;}
    virtual uint32_t getCurrentRollingAverage();
    virtual bool getNodeIDCheck();
    virtual bool getNewSensorValueCheck();
    virtual bool getNewSensorConversionCheck();

    virtual void setNodeIDCheck(bool updatedNodeIDCheck);
    virtual void setState(SensorState newState);
    virtual void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros);
    virtual void linearConversion();
    virtual void exponentialMovingAverage();
    virtual void setTargetValue(float targetValueIn);

    virtual float getEMAConvertedValue();
    virtual float getIntegralSum();
    virtual float getLinRegSlope();

    // constructor 1,
    //MCU_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, uint32_t setSampleRateSlowMode, uint32_t setSampleRateMedMode, uint32_t setSampleRateFastMode, bool internalMCUTemp, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off, bool setNodeIDCheck = false, bool setNewSensorValueCheck = false);
    // constructor 2, define attributes for conversions, gui updates, et cetera
    //SENSOR();

    /* // Access functions defined in place
    uint32_t getSensorID(){return sensorID;}
    uint32_t getSensorNodeID(){return sensorNodeID;}
    uint32_t getADCinput(){return ADCinput;}
    uint32_t getCurrentSampleRate(){return currentSampleRate;}
    uint32_t getCurrentRawValue(){return currentRawValue;}
    float getCurrentConvertedValue(){return currentConvertedValue;}
    float getEMAConvertedValue(){return newEMAOutput;}
    uint16_t getCANTimestamp(){return currentCANtimestamp;}
    uint32_t getTimestampSeconds(){return currentTimestampSeconds;}
    uint32_t getTimestampMicros(){return currentTimestampMicros;}
    //uint8_t getCurrentRollingArrayPosition(){return currentRollingArrayPosition;}
    uint32_t getCurrentRollingAverage(){return currentCalibrationValue;}
    bool getNodeIDCheck(){return nodeIDCheck;}
    bool getNewSensorValueCheck(){return newSensorValueCheck;}
    bool getNewSensorConversionCheck(){return newConversionCheck;}


    // further fuctions defined in SensorClass.cpp
    void begin();                     // run in setup to get pins going
    
    // set functions, allows the setting of a variable
    void setState(SensorState newState) {sensorState = newState;} //every time a state is set, the timer should reset

    // set the Node ID Check bool function
    void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}

    void setCurrentRawValue(uint32_t updateCurrentRawValue){currentRawValue = updateCurrentRawValue;}

    void setNewSensorValueCheck(bool updateNewSensorValueCheck){newSensorValueCheck = updateNewSensorValueCheck;}

    void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

    void setCANTimestamp(uint16_t CANTimestamp){currentCANtimestamp = CANTimestamp;}
    
    void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros){currentTimestampSeconds = timestampSeconds; currentTimestampMicros = timestampMicros;}

    //void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {currentSampleRate = updateCurrentSampleRate; newSensorValueCheck = true; newConversionCheck = false;}
    void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {currentSampleRate = updateCurrentSampleRate;}

    void resetTimer();                // resets timer to zero

    void read(ADC* adc);              // updates currentRawValue with current reading, using an activated ADC object

    void stateOperations();

    void linearConversion();          //Runs a linear sensor conversion 

    void exponentialMovingAverage(); */

/*     //void setRollingSensorArrayRaw(uint8_t arrayPosition, uint16_t sensorValueToArray)
    void setRollingSensorArrayRaw(uint8_t arrayPosition, uint16_t sensorValueToArray)
      {
        rollingSensorArrayRaw[arrayPosition] = sensorValueToArray;
        arrayPosition++;
      } */

/*     void setCurrentCalibrationValue()
    {
    for (size_t i = 0; i < 10; i++)
    {
      currentRunningSUM = currentRunningSUM + rollingSensorArrayRaw[i];
    }
    currentCalibrationValue = currentRunningSUM / 10;
    } */

};

#endif