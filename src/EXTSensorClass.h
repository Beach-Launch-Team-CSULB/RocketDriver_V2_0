#ifndef EXTSENSORCLASS_H
#define EXTSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "SensorStates.h"
#include "SensorClass.h"
#include "fluidSystemSimulation.h"
#include "ALARAUtilityFunctions.h"

//using std::string;

//Declaring setup of the ADC itself for main to find it
void MCUADCSetup();

///// MOVE THIS STUFF FOR NEW POLYMORPHISM SENSOR STRUCTURE /////
// enum for holding sensor types
/* enum SensorType
{
  pt,
  loadcellOneWireRead,
  KtypeTC,
  TtypeTC,
  rtd,
}; */

// enum for holding ADC input types, may not use this way
enum ADCType
{
  TeensyMCUADC, //built in ADC
  ADS1258,  //not in use yet
  ADS1263,  //not in use yet
  simulatedInput, //for simulated sensor inputs
};


class EXT_SENSOR : public SENSORBASE
{
  private:
    const uint32_t sensorID;
    const uint32_t sensorNodeID;                      // NodeID the valve is controlled by
    ADCType sensorSource = TeensyMCUADC;  //default source here is Teensy ADC
    //const string sens_name;           //your own name for sensor to reference it
    SensorState sensorState;
    uint8_t ADCinput;               //the input that will be read for this sensor that will get used in the ADC read main loop
    const uint32_t sampleRateSlowMode = 1;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateMedMode = 10;         //the sample rate this given sensor will be read at
    const uint32_t sampleRateFastMode = 100;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateCalibrationMode = 10;        //the sample rate this given sensor will be read at
    uint32_t currentSampleRate = 10;
    elapsedMicros timer;                      // timer for sensor timing operations
    uint32_t currentRawValue{};               // holds the current value for the sensor
    bool newSensorValueCheck;                      // Is the current raw value a new read that hasn't been sent yet?
    uint16_t currentCANtimestamp = 0;
    uint32_t currentTimestampSeconds = 0;
    uint32_t currentTimestampMicros = 0;
    uint32_t priorTimestampSeconds = 0;
    uint32_t priorTimestampMicros = 0;
    //const uint8_t bitDepth;                   // bit depth of the sample, for output chopping?
    bool nodeIDCheck;                           // Whether this object should operate on this node
    bool internalMCUTemp;                       // Is this sensor the MCU internal temp
    
    float currentConvertedValue{};
    float priorConvertedValue{};
    bool newConversionCheck;                      // Is the current raw value a new read that hasn't been sent yet?
    
    float linConvCoef1_m;                     // Base calibration coefficients
    float linConvCoef1_b;                     // Base calibration coefficients
    float linConvCoef2_m;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef2_b;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    //uint16_t rollingSensorArrayRaw[10];       // Array for doing averages of readings
    //uint8_t currentRollingArrayPosition = 0;
    uint32_t currentCalibrationValue{};               // holds the current value for the sensor
    //uint32_t currentRunningSUM = 0;
    bool EMA = true;  //needs a set function still
    float priorEMAOutput = 0;
    float alphaEMA = 0.7;
    float newEMAOutput = 0;

    bool enableIntegralCalc = false;
    bool enableLinearRegressionCalc = false;
    float currentIntegralSum = 0;
    float currentLinReg_a1 = 0;
    uint32_t regressionSamples = 5;
    float convertedValueArray[5+3] = {};  //should be the same size as regression samples +3 for rolling array index stuff
    float timeStep = 0.01; //timeStep in seconds
    float targetValue = 0;
  FluidSystemSimulation &fluidSim;

  public:
    void begin();                     // run in setup to get pins going
    void read();              // updates currentRawValue with current reading, using an activated ADC object
    void stateOperations();
    
    // constructor 1 - standard MCU external ADC read
    EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, uint32_t setSampleRateSlowMode, uint32_t setSampleRateMedMode, uint32_t setSampleRateFastMode, float setLinConvCoef1_m = 1, float setLinConvCoef1_b = 0, float setLinConvCoef2_m = 1, float setLinConvCoef2_b = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off, bool setNodeIDCheck = false, bool setNewSensorValueCheck = false, bool setNewConversionCheck = false);
    // constructor 2 - simulated sensor object
    EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, ADCType setSensorSource = simulatedInput);

    // Access functions defined in place
    uint32_t getSensorID(){return sensorID;}
    uint32_t getSensorNodeID(){return sensorNodeID;}
    uint32_t getADCinput(){return ADCinput;}
    uint32_t getCurrentSampleRate(){return currentSampleRate;}
    uint32_t getCurrentRawValue(){return currentRawValue;}
    float getCurrentConvertedValue(){return currentConvertedValue;}
    uint16_t getCANTimestamp(){return currentCANtimestamp;}
    uint32_t getTimestampSeconds(){return currentTimestampSeconds;}
    uint32_t getTimestampMicros(){return currentTimestampMicros;}
    uint32_t getCurrentRollingAverage(){return currentCalibrationValue;}
    bool getNodeIDCheck(){return nodeIDCheck;}
    bool getNewSensorValueCheck(){return newSensorValueCheck;}
    bool getNewSensorConversionCheck(){return newConversionCheck;}
    bool getEnableLinearRegressionCalc(){return enableLinearRegressionCalc;}
    
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

    void setCurrentRawValue(uint32_t updateCurrentRawValue){currentRawValue = updateCurrentRawValue;}

    void setNewSensorValueCheck(bool updateNewSensorValueCheck){newSensorValueCheck = updateNewSensorValueCheck;}

    void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}

    void setCANTimestamp(uint16_t CANTimestamp){currentCANtimestamp = CANTimestamp;}
    
    void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros)
      {
        priorTimestampSeconds = currentTimestampSeconds;  //shifts the previous current into prior variables
        priorTimestampMicros = currentTimestampMicros;
        currentTimestampSeconds = timestampSeconds;       //sets the new current timestamps from input arguments
        currentTimestampMicros = timestampMicros;
      }

    //void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {currentSampleRate = updateCurrentSampleRate; newSensorValueCheck = true; newConversionCheck = false;}
    void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {currentSampleRate = updateCurrentSampleRate;}

    void setTargetValue(float targetValueIn){targetValue = targetValueIn;}

    void resetTimer();                // resets timer to zero

    //void read(ADC* adc);              // updates currentRawValue with current reading, using an activated ADC object

    void linearConversion();          //Runs a linear sensor conversion 

    void exponentialMovingAverage();

    void initializeLinReg(uint8_t arraySizeIn);

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