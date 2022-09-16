#ifndef EXTDIGITALDIFFLCSENSORCLASS_H
#define EXTDIGITALDIFFLCSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "SensorStates.h"
#include "SensorClass.h"
#include "fluidSystemSimulation.h"
#include "ALARAUtilityFunctions.h"

class DIG_LC_SENSOR : public SENSORBASE
{
  private:
    const uint32_t sensorID;
    const uint32_t sensorNodeID;                      // NodeID the valve is controlled by
    ADCType sensorSource = TeensyMCUADC;  //default source here is Teensy ADC
    //const string sens_name;           //your own name for sensor to reference it
    SensorState sensorState;
    uint8_t ADCinput1;               //the input that will be read for this sensor that will get used in the ADC read main loop
    uint8_t ADCinput2;               //the input that will be read for this sensor that will get used in the ADC read main loop
    const uint32_t sampleRateSlowMode_Default = 1;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateMedMode_Default = 10;         //the sample rate this given sensor will be read at
    const uint32_t sampleRateFastMode_Default = 200;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
    const uint32_t conversionSendRate_Default = 100;        //the sample rate this given sensor will be read at
    uint32_t sampleRateSlowMode;        //the sample rate this given sensor will be read at
    uint32_t sampleRateMedMode;         //the sample rate this given sensor will be read at
    uint32_t sampleRateFastMode;        //the sample rate this given sensor will be read at
    uint32_t sampleRateCalibrationMode;        //the sample rate this given sensor will be read at
    uint32_t conversionSendRate;        //the sample rate this given sensor will be read at
    uint32_t currentSampleRate = 10;
    elapsedMicros timer;                      // timer for sensor timing operations
    uint32_t currentRawValue1{};               // holds the current value for the sensor
    uint32_t currentRawValue2{};               // holds the current value for the sensor
    uint32_t currentRawDiffValue{};               // holds the current value for the sensor
    bool newSensorValueCheck_CAN = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newSensorConvertedValueCheck_CAN = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newSensorValueCheck_Log = false;                      // Is the current raw value a new read that hasn't been sent yet?
    uint16_t currentCANtimestamp = 0;
    uint32_t currentTimestampSeconds = 0;
    uint32_t currentTimestampMicros = 0;
    uint32_t priorTimestampSeconds = 0;
    uint32_t priorTimestampMicros = 0;
    //const uint8_t bitDepth;                   // bit depth of the sample, for output chopping?
    bool nodeIDCheck = false;                           // Whether this object should operate on this node
    bool internalMCUTemp;                       // Is this sensor the MCU internal temp
    
    float currentConvertedValue{};
    float priorConvertedValue{};
    bool newConversionCheck = false;                      // Is the current raw value a new read that hasn't been sent yet?

    float linConvCoef1_m_Default;                     // Base calibration coefficients
    float linConvCoef1_b_Default;                     // Base calibration coefficients
    float linConvCoef2_m_Default;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef2_b_Default;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef1_m;                     // Base calibration coefficients
    float linConvCoef1_b;                     // Base calibration coefficients
    float linConvCoef2_m;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef2_b;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    //uint16_t rollingSensorArrayRaw[10];       // Array for doing averages of readings
    //uint8_t currentRollingArrayPosition = 0;
    uint32_t currentCalibrationValue{};               // holds the current value for the sensor
    //uint32_t currentRunningSUM = 0;
    bool EMA_Default = true;  //needs a set function still
    bool EMA;  //needs a set function still
    float priorEMAOutput = 0;
    float alphaEMA_Default = 0.7; //1 is no weight to old values, 0 has no weight to new value and will brick
    float alphaEMA;
    float newEMAOutput = 0;

    bool enableIntegralCalc = false;
    bool enableLinearRegressionCalc = true; //not currently using, linreg only calculates when get func requests it
    float maxIntegralSum_Default;
    float minIntegralSum_Default;
    float maxIntegralSum;
    float minIntegralSum;
    float currentIntegralSum = 0;
    float currentLinReg_a1 = 0;
    const uint32_t regressionSamples_Default = 5;
    uint32_t regressionSamples;
    float convertedValueArray[5+3] = {};  //should be the same size as regression samples +3 for rolling array index stuff
    float timeStep = 0.01; //timeStep in seconds, placeholder value
    float targetValue = 0;
  
  FluidSystemSimulation &fluidSim;

  public:
    bool pullTimestamp = false;
    void begin();                     // run in setup to get pins going
    void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    void stateOperations();
    
    // constructor 1 - standard MCU external ADC read
    DIG_LC_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput1, uint8_t setADCinput2, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, uint32_t setConversionSendRate_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);

    // Access functions defined in place
    uint32_t getSensorID(){return sensorID;}
    uint32_t getSensorNodeID(){return sensorNodeID;}
    uint32_t getADCinput(){return ADCinput1;}
    uint32_t getADCinput2(){return ADCinput2;}
    uint32_t getCurrentSampleRate(){return currentSampleRate;}
    uint32_t getCurrentRawValue(){return currentRawValue1;}
    //uint32_t getCurrentRawValue(){return currentRawValue2;}
    uint32_t getCurrentRawValue(bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} return currentRawValue1;} //reads and clears new value bool
    uint32_t getCurrentRawValue(bool input1, bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} if (!input1) {return currentRawValue1;} else{return currentRawValue2;};} //reads and clears new value bool
    float getCurrentConvertedValue(){return currentConvertedValue;}
    //float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newSensorConvertedValueCheck_CAN = false;} return currentConvertedValue;} //reads and clears new value bool
    float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newConversionCheck = false;} return currentConvertedValue;} //reads and clears new value bool
    uint16_t getCANTimestamp(){return currentCANtimestamp;}
    uint32_t getTimestampSeconds(){return currentTimestampSeconds;}
    uint32_t getTimestampMicros(){return currentTimestampMicros;}
    uint32_t getCurrentRollingAverage(){return currentCalibrationValue;}
    ADCType getADCtype(){return sensorSource;}
    bool getNodeIDCheck(){return nodeIDCheck;}
    bool getNewSensorValueCheckCAN(){return newSensorValueCheck_CAN;}
    bool getNewSensorValueCheckLog(){return newSensorValueCheck_Log;}
    bool getNewSensorConversionCheck(){return newConversionCheck;}
    bool getEnableLinearRegressionCalc(){return enableLinearRegressionCalc;}
    bool getEnableIntegralCalc(){return enableIntegralCalc;}
    float getMaxIntegralSum(){return maxIntegralSum;}
    float getMinIntegralSum(){return minIntegralSum;}

    float getEMAConvertedValue(){return newEMAOutput;}
    float getIntegralSum(){return currentIntegralSum;}
    float getLinRegSlope(){currentLinReg_a1 = linearRegressionLeastSquared_PID(); return currentLinReg_a1;}

    // further fuctions defined in SensorClass.cpp
/*     void begin();                     // run in setup to get pins going

    void read();              // updates currentRawValue with current reading, using an activated ADC object

    void stateOperations();
 */    
    // set functions, allows the setting of a variable
    //void setState(SensorState newState) {sensorState = newState;} //every time a state is set, the timer should reset
    void setState(SensorState newState);

    // set the Node ID Check bool function
    void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}

    //Why is set raw value here again?
    void setCurrentRawValue(uint32_t updateCurrentRawValue){currentRawValue1 = updateCurrentRawValue;}
    //resets both the CAN and log bools for when new sample is read
    void setNewSensorValueCheck(bool updateNewSensorValueCheck){newSensorValueCheck_CAN = updateNewSensorValueCheck; newSensorValueCheck_Log = updateNewSensorValueCheck;}

    void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

    void setCANTimestamp(uint16_t CANTimestamp){currentCANtimestamp = CANTimestamp;}
    
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

    //void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {currentSampleRate = updateCurrentSampleRate; newSensorValueCheck = true; newConversionCheck = false;}
    void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {currentSampleRate = updateCurrentSampleRate;}
    
    void setSampleRateSlowMode(uint32_t updateSampleRateSlowMode) {if(updateSampleRateSlowMode<=2000){sampleRateSlowMode = updateSampleRateSlowMode;}}
    void setSampleRateMedMode(uint32_t updateSampleRateMedMode) {if(updateSampleRateMedMode<=2000){sampleRateMedMode = updateSampleRateMedMode;}}
    void setSampleRateFastMode(uint32_t updateSampleRateFastMode) {if(updateSampleRateFastMode<=2000){sampleRateFastMode = updateSampleRateFastMode;}}
    void setAlphaEMA(float alphaEMAIn){if(alphaEMAIn >0 && alphaEMAIn <=1){alphaEMA = alphaEMAIn;}}
    void setMaxIntegralSum(float maxIntegralSumIn){maxIntegralSum = maxIntegralSumIn;}
    void setMinIntegralSum(float minIntegralSumIn){maxIntegralSum = minIntegralSumIn;}
    
    //void setRegressionSamples():???
    
    void setTargetValue(float targetValueIn){targetValue = targetValueIn;}

    void resetTimer();                // resets timer to zero
    // reset all configurable settings to defaults
    void resetAll();

    //void read(ADC* adc);              // updates currentRawValue with current reading, using an activated ADC object

    void linearConversion();          //Runs a linear sensor conversion 

    void exponentialMovingAverage();

    void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment

    void setEnableIntegralCalc(bool setEnableIn){enableIntegralCalc = setEnableIn;}

    void resetIntegralCalc(bool resetBoolIn, float integralCalcIn = 0){if(resetBoolIn){currentIntegralSum = integralCalcIn;}}  //resets the integral sum, default arg zeros it

    float linearRegressionLeastSquared_PID();

    void accumulatedI_float();

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
    //};
};

// need to add differential read toggle somehow 
// - differential boolean variable that allows second input to be chosen or defaulted to correct option
// need to add a way to set other SENSOR types like the RTD sensors over I2C (we'd probably want multiple classes. ADCsensors, I2C sensors, SPI sensors etc - Mat)
// - maybe not the right call to roll into this? Hmm. Need to establish use of SENSOR class with sample rates and real read/sends to see what is better
// That will set me up for incorporating the external ADCs later



#endif