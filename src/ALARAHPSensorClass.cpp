#include "ALARAHPSensorClass.h"
#include <Arduino.h>
#include <ADC.h>
#include <array>

//#include <ADC_util.h>

//using std::string;

/* ///// ADC /////
ADC* adc = new ADC();

void MCUADCSetup()
{ 
//Ideally get some conditionals here for which MCU it is so this is compatible at least also with Teensy LC

///// ADC0 /////
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 or ADC_REFERENCE::REF_EXT.
  //adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->adc0->setReference(ADC_REFERENCE::REF_1V2);
  adc->adc0->setAveraging(4);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // change the sampling speed
  adc->adc0->recalibrate();

///// ADC1 /////
  adc->adc1->setReference(ADC_REFERENCE::REF_1V2);
  adc->adc1->setAveraging(4);                                    // set number of averages
  adc->adc1->setResolution(16);                                   // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // change the sampling speed
  adc->adc1->recalibrate();

} */


// Initializer 1
/* ALARAHP_SENSOR::ALARAHP_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off)
                : sensorID{setSensorID}, sensorNodeID{setSensorNodeID}, ADCinput{setADCinput}, sampleRateSlowMode_Default{setSampleRateSlowMode_Default}, sampleRateMedMode_Default{setSampleRateMedMode_Default}, sampleRateFastMode_Default{setSampleRateFastMode_Default}, linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default}, linConvCoef2_m_Default{setLinConvCoef2_m_Default}, linConvCoef2_b_Default{setLinConvCoef2_b_Default}, currentSampleRate{setCurrentSampleRate}, sensorState{setSensorState}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  linConvCoef1_m = linConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default;

  EMA = EMA_Default;
  alphaEMA = alphaEMA_Default;
  regressionSamples = regressionSamples_Default;
} */

// Initializer 2
ALARAHP_SENSOR::ALARAHP_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Slow)
                : sensorID{setSensorID}, sensorNodeID{setSensorNodeID}, ADCinput{setADCinput}, linConvCoef1_m_Default{setLinConvCoef1_m_Default}, linConvCoef1_b_Default{setLinConvCoef1_b_Default}, linConvCoef2_m_Default{setLinConvCoef2_m_Default}, linConvCoef2_b_Default{setLinConvCoef2_b_Default}, currentSampleRate{setCurrentSampleRate}, sensorState{setSensorState}
{
  // setting stuff to defaults at initialization
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  linConvCoef1_m = linConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default;

  EMA = EMA_Default;
  alphaEMA = alphaEMA_Default;
  regressionSamples = regressionSamples_Default;
}


void ALARAHP_SENSOR::begin()
{
    if (nodeIDCheck)
    {
        pinMode(ADCinput, INPUT);
    }
}

void ALARAHP_SENSOR::resetTimer()
{
    timer = 0;
}

void ALARAHP_SENSOR::resetAll()
{
  //
  sampleRateSlowMode = sampleRateSlowMode_Default;
  sampleRateMedMode = sampleRateMedMode_Default;
  sampleRateFastMode = sampleRateFastMode_Default;
  sampleRateCalibrationMode = sampleRateCalibrationMode_Default;

  linConvCoef1_m = linConvCoef1_m_Default;
  linConvCoef1_b = linConvCoef1_b_Default;
  linConvCoef2_m = linConvCoef2_m_Default;
  linConvCoef2_b = linConvCoef2_b_Default;

  EMA = EMA_Default;
  alphaEMA = alphaEMA_Default;
}

void ALARAHP_SENSOR::setState(SensorState newState) 
{
    sensorState = newState;
}

void ALARAHP_SENSOR::read(ADC& adc)
{
    //Add in sample rate code here to check if a sensor is up to be read
    //This is also where alternate ADC sources would be used - I do have the RTD sensors over ITC right now
    //I'll have to change how it's written though, right now it's ADC* adc which is specific to Teensy MCU ADC
        if (currentSampleRate != 0)     //math says no divide by zero, use separate conditional for sample rate of 0
        {
        if (timer >= (1000000/currentSampleRate))   // Divides 1 second in microseconds by current sample rate in Hz
            {
                    currentRawValue = adc.analogRead(ADCinput);
                    pullTimestamp = true;
                    //setRollingSensorArrayRaw(currentRollingArrayPosition, currentRawValue);
                    /////linear conversions here, y = m*x + b
                    // This automatically stores converted value for the on board nodes
                    priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
                    currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;
                    //writeToRollingArray(convertedValueArray, currentConvertedValue);
                    exponentialMovingAverage();
                    //accumulatedI_float();

                newSensorValueCheck_CAN = true;
                newSensorValueCheck_Log = true;
                newSensorConvertedValueCheck_CAN = true;
                //newSensorValueCheck = false;
                newConversionCheck = true;
                //Serial.println("newSensorinREADafter");
                //Serial.println(newSensorValueCheck);
                timer = 0;
            }
        
      }

}

void ALARAHP_SENSOR::stateOperations()
{

    switch (sensorState)
    {
    case SensorState::Off:
        setCurrentSampleRate(0);
        timeStep = 1; //timeStep in seconds - shitty hack to make it not brick to a nan from dividing by zero
        break;
    case SensorState::Slow:
        setCurrentSampleRate(sampleRateSlowMode);
        timeStep = 1/sampleRateSlowMode;
        break;
    case SensorState::Medium:
        setCurrentSampleRate(sampleRateMedMode);
        timeStep = 1/sampleRateMedMode; //timeStep in seconds
        break;
    case SensorState::Fast:
        setCurrentSampleRate(sampleRateFastMode);
        timeStep = 1/sampleRateFastMode; //timeStep in seconds
        break;
    // All other states require no action
    default:
        break;
    }
}

void ALARAHP_SENSOR::linearConversion()
{
    /////linear conversions here, y = m*x + b
    //if (newSensorValueCheck && newConversionCheck == false)
    if (newConversionCheck == false)
    {
    //priorConvertedValue = currentConvertedValue; //shifts previous converted value into prior variable
    currentConvertedValue = linConvCoef1_m*currentRawValue + linConvCoef1_b;    //Initial Calibration
    currentConvertedValue = linConvCoef2_m*currentConvertedValue + linConvCoef2_b;    //Secondary Calibration
    newConversionCheck = true;

/*             Serial.print("sensorID: ");
            Serial.print(sensorID);
            Serial.print(", currentRawValue: ");
            Serial.println(currentRawValue);
            Serial.print(", currentConvertedValue: ");
            Serial.println(currentConvertedValue); */
    }
}

void ALARAHP_SENSOR::exponentialMovingAverage()
{
  //function written to accept and return floats
  //alpha must be between 0 and 1, force overflows to max and min weights
  
  //Serial.print("alphaEMA");
  //Serial.println(alphaEMA);
  if (EMA)  //only run if EMA bool is true
  {
    // bounds EMA between 0 and 1 for valid formula
    if (alphaEMA >= 1)
    {
        alphaEMA = 1;
    }
    else if (alphaEMA <= 0)
    {
        alphaEMA = 0;
    }
    //quick maffs
    newEMAOutput = (alphaEMA*currentConvertedValue) + ((1 - alphaEMA)*(priorEMAOutput));
    priorEMAOutput = newEMAOutput;
  }
  else //EMA calc still runs this way but with no computation, just setting the values. Could possibly cut even this for performance.
  {
    newEMAOutput = currentConvertedValue;
    priorEMAOutput = newEMAOutput;
  }
}


void ALARAHP_SENSOR::initializeLinReg(uint8_t arraySizeIn)
{
    if(!enableLinearRegressionCalc) //only initializes the array if it wasn't already
    {
    enableLinearRegressionCalc = true;
    //delete[]convertedValueArray;  //destroys the old version of the array
    //float convertedValueArray[arraySizeIn+3] = {};
    //rolling array setup
    convertedValueArray[0] = {3};
    convertedValueArray[1] = {3};
    //convertedValueArray[2] = {static_cast<float>(arraySizeIn)};
    }

/*   uint32_t arrayIndexFirstValueLinReg = 0;
  uint32_t arrayWrapSizeLinReg = 0;
  uint32_t arrayMostRecentPositionLinReg = 0;
  uint32_t regression_n = 0;
  uint32_t sizeInputArrayLinReg = 0;

  float sumX = 0;
  float sumY = 0;
  float sumXX = 0;
  float sumXY = 0;
  float denLeastSquare = 0;
  float a0LeastSquare = 0;
  float a1LeastSquare = 0; */
}


float ALARAHP_SENSOR::linearRegressionLeastSquared_PID()
{
  uint32_t arrayIndexFirstValueLinReg = 0;
  uint32_t arrayWrapSizeLinReg = 0;
  uint32_t arrayMostRecentPositionLinReg = 0;
  uint32_t regression_n = 0;
  uint32_t sizeInputArrayLinReg = 0;

  float sumX = 0;
  float sumY = 0;
  float sumXX = 0;
  float sumXY = 0;
  float denLeastSquare = 0;
  float a0LeastSquare = 0;
  float a1LeastSquare = 0;

  sumX = 0;
  sumY = 0;
  sumXX = 0;
  sumXY = 0;
  denLeastSquare = 0;
  a0LeastSquare = 0;
  a1LeastSquare = 0;
  
  
  // Version of linear regression simplified for finding the recent slope for a PID controller
  // assumes fixed time steps, time is X, controller variable Y
  // !!!!! - Function is built to expect arrays in format of:
  // !!!!! - index[0] = first index with a value entry
  // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
  // !!!!! - index[2] = size of value entries
  // Not sure if I've updated the math to use the first value in the numerical part instead of just manually subtracting to make it work for when it's 3
  arrayIndexFirstValueLinReg = static_cast<uint32_t>(convertedValueArray[0]+0.5);
  arrayMostRecentPositionLinReg = static_cast<uint32_t>(convertedValueArray[1]+0.5);
  sizeInputArrayLinReg = static_cast<uint32_t>(convertedValueArray[2]+0.5);
  
    // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
  if (sizeInputArrayLinReg < regressionSamples)
  {
    regression_n = sizeInputArrayLinReg;
  }
  else regression_n = regressionSamples;
  // determine the overwrap value, if any
  //arrayWrapSizeLinReg =  (-1) * ((arrayMostRecentPositionLinReg - regression_n) - 1); //old array method
  arrayWrapSizeLinReg =  regression_n - ((arrayMostRecentPositionLinReg) - (arrayIndexFirstValueLinReg) + 1);

  // calculate the sum terms;
  // 
    //Serial.print("timeStep: ");
    //Serial.println(timeStep);
    timeStep = 0.01;
//dont think I need below with new methods
/*     if (arrayWrapSizeLinReg <= 0)    // when there is no wrap required, calculated value will be zero or negative. Set to zero.
    {
      arrayWrapSizeLinReg = 0;
    } */
    //Serial.print("overwrap after zero set: ");
    //Serial.println(arrayWrapSizeLinReg);
  if (arrayWrapSizeLinReg > 0)  //only true if there are enough array values to use to wrap the end of the array
  {
    for (int i = arrayMostRecentPositionLinReg; i > (arrayIndexFirstValueLinReg - 1); i--)
    {
      sumX = sumX + ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep);
      sumXX = sumXX + (((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep)*((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
      sumY = sumY + convertedValueArray[i];
      sumXY = sumXY + (convertedValueArray[i] * ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
/*       Serial.print("DOES THIS EVER HAPPEN1: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println((i - (arrayMostRecentPositionLinReg - regression_n + 1)));
 */
    }  

    for (int i = (sizeInputArrayLinReg + arrayIndexFirstValueLinReg - 1); i > (sizeInputArrayLinReg + arrayIndexFirstValueLinReg - 1 - arrayWrapSizeLinReg); i--)
    {
      sumX = sumX + ((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3)*timeStep);
      sumXX = sumXX + (((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3)*timeStep)*((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3)*timeStep));
      sumY = sumY + convertedValueArray[i];
      sumXY = sumXY + (convertedValueArray[i] * ((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3)*timeStep));
/*       Serial.print("DOES THIS EVER HAPPEN2: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3));
 */
    }  
  }
  else
  {
    for (int i = arrayMostRecentPositionLinReg; i > (arrayMostRecentPositionLinReg - regression_n); i--)
    {
      sumX = sumX + ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep);
      sumXX = sumXX + (((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep)*((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
      sumY = sumY + convertedValueArray[i];
      sumXY = sumXY + (convertedValueArray[i] * ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
/*       Serial.print("DOES THIS EVER HAPPEN: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println((i - (arrayMostRecentPositionLinReg - regression_n + 1)));
 */    
    }
  }
/*   Serial.print("sumX: ");
  Serial.println(sumX,8);
  Serial.print("sumY: ");
  Serial.println(sumY,8);
  Serial.print("sumXX: ");
  Serial.println(sumXX,8);
  Serial.print("sumXY: ");
  Serial.println(sumXY,8); */

  // calculate the denominator term
  denLeastSquare = regression_n*sumXX - (sumX * sumX);
  //Serial.print("den: ");
  //Serial.println(denLeastSquare,5);
  // calculate the a1 term, which is the slope
  a1LeastSquare = ((regression_n*sumXY) - (sumX*sumY))/denLeastSquare;
  // calculate the a0 term, which is the linear offset
  // NOT USED IN PID VERSION
  // a0LeastSquare = ((sumXX*sumY) - (sumXY*sumX))/denLeastSquare;
  return a1LeastSquare;
}

void ALARAHP_SENSOR::accumulatedI_float()
{
float accumIfuncOutput = 0;
float timeStepAccumI = 0;
    if (enableIntegralCalc)
    {
        //timeStepAccumI = (currentTimestampSeconds - priorTimestampSeconds) + ((currentTimestampMicros - priorTimestampMicros)*1000000); //calculates timestep between samples in S
        timeStepAccumI = timer/float(1000000);
        //timeStepAccumI = 0.01;
/*         Serial.print(" ID: ");
        Serial.print(sensorID);
        Serial.print(" timer: ");
        Serial.println(timer,10);
        Serial.print(" timeStepAccumI: ");
        Serial.println(timeStepAccumI,10); */
        // trapazoid method for area under the curve using current and previous values as the end points
        /* Serial.print("currentInputValue");
        Serial.println(currentInputValue);
        Serial.print("previousInputValue");
        Serial.println(previousInputValue);
        Serial.print("accumIfuncOutput");
        Serial.println(accumIfuncOutput,10);
        Serial.print("timeStepAccumI");
        Serial.println(timeStepAccumI,10); */

        currentIntegralSum += timeStepAccumI * (((currentConvertedValue - targetValue) + (priorConvertedValue - targetValue))/2);
    }
}
