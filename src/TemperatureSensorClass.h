#ifndef TEMPERATURESENSORCLASS_H
#define TEMPERATURESENSORCLASS_H

#include <Arduino.h>
#include <ADC.h>
#include "SensorStates.h"
#include "Adafruit_MCP9808.h"
#include "ALARAUtilityFunctions.h"

//#include "thermocoupleT_K.h"
#pragma once

enum TCType
{
    K_Type,
    T_Type,
};

class RTD_BREAKOUT
{
    private:
        const uint32_t sensorID;
        const uint32_t sensorNodeID;                        // NodeID the sensor is controlled by
        uint8_t I2Caddress;                                 // 0x18 default, up to 0x1F
        uint8_t resolution;                                 // 0, 1, 2, 3
        // Minimal read periods for each Resultion setting, in Millis
        uint32_t sampleTimeCurrent = 250;
        const uint32_t sampleTimeResolution0 = 30;
        const uint32_t sampleTimeResolution1 = 65;
        const uint32_t sampleTimeResolution2 = 130;
        const uint32_t sampleTimeResolution3 = 250;
        elapsedMillis timer;
        int16_t convertedReadC = 0;
        int16_t convertedReadF = 0;
        Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();    
        bool sensorInitialized = false;
    public:
        // Constructor
        RTD_BREAKOUT(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setI2Caddress, uint8_t setResolution);

        //get functions
        uint32_t getSensorID(){return sensorID;}
        uint32_t getSensorNodeID(){return sensorNodeID;}
        elapsedMillis getTimer(){return timer;}
        int16_t getConvertedValueC(){return convertedReadC;}
        int16_t getConvertedValueF(){return convertedReadF;}
        
        // whatever
        void begin();
        void read();
        void stateOperations();

};

class THERMOCOUPLE
{
    private:
        const uint32_t sensorID;
        const uint32_t sensorNodeID;
        uint8_t ADCinput1;
        uint8_t ADCinput2;
        uint16_t refVoltage;    // in millivolts as int
        TCType tc;
        int16_t priorColdJunctionTempC;
        int16_t coldJunctionTempC;
        uint32_t oversampleCounter = 0;
        uint32_t currentTimestampSeconds = 0;
        uint32_t currentTimestampMicros = 0;
        uint32_t priorTimestampSeconds = 0;
        uint32_t priorTimestampMicros = 0;
        bool nodeIDCheck = false;                           // Whether this object should operate on this node

        RTD_BREAKOUT& tempsensor;
        elapsedMillis RTDtimer;
        elapsedMillis timer;
        bool newRTD = false;
        uint32_t currentRawValue1{};               // holds the current value for the sensor
        uint32_t currentRawValue2{};               // holds the current value for the sensor

        float currentConvertedValue{};
        float priorConvertedValue{};
        bool EMA_Default = true;  //needs a set function still
        bool EMA;  //needs a set function still
        float priorEMAOutput = 0;
        float alphaEMA_Default = 0.7; //1 is no weight to old values, 0 has no weight to new value and will brick
        float alphaEMA;
        float newEMAOutput = 0;

        const uint32_t regressionSamples_Default = 5;
        uint32_t regressionSamples = 5;
        float convertedValueArray[5+3] = {};  //should be the same size as regression samples +3 for rolling array index stuff
        float timeStep = 0.01; //timeStep in seconds, should be set based on sample rate to correct value this is a placeholder value

    public:
        bool pullTimestamp = false;
        uint32_t getSensorID(){return sensorID;}
        uint32_t getSensorNodeID(){return sensorNodeID;}


        void begin();
        void read(ADC& adc);
        void stateOperations();

        bool getNodeIDCheck(){return nodeIDCheck;}
    
        // set functions, allows the setting of a variable
        void setState(SensorState newState);
        // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
        
        //void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

        void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros)
        {
            if (pullTimestamp)
            {
            priorTimestampSeconds = currentTimestampSeconds;  //shifts the previous current into prior variables
            priorTimestampMicros = currentTimestampMicros;
            currentTimestampSeconds = timestampSeconds;       //sets the new current timestamps from input arguments
            currentTimestampMicros = timestampMicros;
            pullTimestamp = false;
            }
        }

        void setAlphaEMA(float alphaEMAIn){if(alphaEMAIn >0 && alphaEMAIn <=1){alphaEMA = alphaEMAIn;}}
        void resetTimer();                // resets timer to zero
        // reset all configurable settings to defaults
        void resetAll();

        void exponentialMovingAverage();
        float linearRegressionLeastSquared_PID();
        // Constructor
        THERMOCOUPLE(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput1, uint8_t setADCinput2, TCType setTc, RTD_BREAKOUT* setTempsensor, uint16_t setRefVoltage = 1200);
};


#endif