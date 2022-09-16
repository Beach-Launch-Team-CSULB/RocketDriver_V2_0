#ifndef SENSORCLASS_H
#define SENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "SensorStates.h"
#pragma once

// enum for holding ADC input types, may not use this way
enum ADCType
{
  TeensyMCUADC, //built in ADC
  ADS1258,  //not in use yet
  ADS1263,  //not in use yet
  simulatedInput, //for simulated sensor inputs
};


// Base sensor class to hold the various derived classes, should be purely virtual functions
class SENSORBASE
{
  public:
    virtual void begin();                     //
    virtual void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    virtual void stateOperations();

    // Access functions defined in place
    virtual uint32_t getSensorID();
    virtual uint32_t getSensorNodeID();
    virtual uint32_t getADCinput();
    //virtual uint32_t getADCinput(bool input1);
    virtual uint32_t getCurrentSampleRate();
    virtual uint32_t getCurrentRawValue();
    virtual uint32_t getCurrentRawValue(bool resetRawRead);
    //virtual uint32_t getCurrentRawValue(bool input1, bool resetRawRead);
    virtual float getCurrentConvertedValue();
    virtual float getCurrentConvertedValue(bool resetConvertedRead);
    virtual uint16_t getCANTimestamp();
    virtual uint32_t getTimestampSeconds();
    virtual uint32_t getTimestampMicros();
    //virtual uint8_t getCurrentRollingArrayPosition(){return currentRollingArrayPosition;}
    virtual uint32_t getCurrentRollingAverage();
    virtual ADCType getADCtype();
    virtual bool getNodeIDCheck();
    virtual bool getNewSensorValueCheckCAN();
    virtual bool getNewSensorValueCheckLog();
    virtual bool getNewSensorConversionCheck();
    virtual bool getEnableLinearRegressionCalc();
    virtual bool getEnableIntegralCalc();
    virtual float getMaxIntegralSum();
    virtual float getMinIntegralSum();
    virtual void setNodeIDCheck(bool updatedNodeIDCheck);
    virtual void setState(SensorState newState);
    virtual void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros);
    virtual void linearConversion();
    virtual void exponentialMovingAverage();
    virtual void setTargetValue(float targetValueIn);

    virtual float getEMAConvertedValue();
    //virtual float getDeengergizeOffsetValue();
    virtual float getIntegralSum();
    virtual float getLinRegSlope();

    virtual void initializeLinReg(uint8_t arraySizeIn);
    virtual void setEnableIntegralCalc(bool setEnableIn);
    virtual void resetIntegralCalc(bool resetBoolIn, float integralCalcIn = 0);

    virtual void resetAll();
    virtual void setSampleRateSlowMode(uint32_t updateSampleRateSlowMode);
    virtual void setSampleRateMedMode(uint32_t updateSampleRateMedMode);
    virtual void setSampleRateFastMode(uint32_t updateSampleRateFastMode);
    virtual void setAlphaEMA(float alphaEMAIn);
    //virtual void setDeenergizeOffset(ADC& adc, bool outputOverrideIn);

};

#endif