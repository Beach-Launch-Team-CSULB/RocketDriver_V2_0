#include "TemperatureSensorClass.h"




RTD_BREAKOUT::RTD_BREAKOUT(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setI2Caddress, uint8_t setResolution)
                        : sensorID{setSensorID}, sensorNodeID{setSensorNodeID}, I2Caddress{setI2Caddress}, resolution{setResolution}
{
  // Bound check resolution setting
  if (resolution >= 3)
  {
    resolution = 3;
  }
  // Bound check I2C address setting
  if (I2Caddress < 24 || I2Caddress > 31)
  {
    // If address from constructor out of bounds, set to default address
    I2Caddress = 24;
  }
  if (resolution == 3)
  {
  sampleTimeCurrent = sampleTimeResolution3;
  }
  else if (resolution == 2)
  {
  sampleTimeCurrent = sampleTimeResolution2;
  }
  else if (resolution == 1)
  {
  sampleTimeCurrent = sampleTimeResolution1;
  }
  else if (resolution == 0)
  {
  sampleTimeCurrent = sampleTimeResolution0;
  }
  
}

void RTD_BREAKOUT::begin()
{
if(!tempsensor.begin(I2Caddress))
{
  Serial.println("Temp Sensor did not initialize.");
  sensorInitialized = false;
}
else
{
  tempsensor.begin(I2Caddress);
  sensorInitialized = true;
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F
tempsensor.setResolution(resolution);
  // Match the fastest sample requests in the loop to the sample time for chosen setting or it will not return data
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
}


}

void RTD_BREAKOUT::read()
{
  if (sensorInitialized && (timer >= sampleTimeCurrent))
  {
    convertedReadC = static_cast<int16_t>(tempsensor.readTempC()+0.5);
    Serial.print("Temp C: ");
    Serial.print(convertedReadC);
    timer = 0;
  }
}


THERMOCOUPLE::THERMOCOUPLE(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput1, uint8_t setADCinput2, TCType setTc, RTD_BREAKOUT* setTempsensor, uint16_t setRefVoltage)
                  : sensorID{setSensorID}, sensorNodeID{setSensorNodeID}, ADCinput1{setADCinput1}, ADCinput2{setADCinput2}, tc{setTc}, tempsensor{*setTempsensor}, refVoltage{setRefVoltage}
{
  
}

void THERMOCOUPLE::begin()
{
    
    if (nodeIDCheck)
    {
        pinMode(ADCinput1, INPUT);
        pinMode(ADCinput2, INPUT);
        //rolling array setup
        convertedValueArray[0] = {3};
        convertedValueArray[1] = {3};
        convertedValueArray[2] = {static_cast<float>(regressionSamples)};
    }
}

void THERMOCOUPLE::stateOperations()
{
  // Use timers to track whether the RTD has updated and new subsampling period begins for the TC
  if (RTDtimer != tempsensor.getTimer())
  {
    // if the timer is not equal, then make it so and flag to start a new TC averaging period
    RTDtimer = tempsensor.getTimer();
    priorColdJunctionTempC = coldJunctionTempC;
    coldJunctionTempC = tempsensor.getConvertedValueC();
    newRTD = true;
  }
  
}

void THERMOCOUPLE::read(ADC& adc)
{
  if (newRTD)
  {
    // shift the current running raw values into prior values and do conversions on them
    newRTD = false;
  }
  currentRawValue1 = adc.analogRead(ADCinput1);
  currentRawValue2 = adc.analogRead(ADCinput2);
  Serial.print(" sensorID: ");
  Serial.print(sensorID);
  Serial.print(" priorRTD C: ");
  Serial.print(priorColdJunctionTempC);
  Serial.print(" RTD C: ");
  Serial.print(coldJunctionTempC);
  Serial.print(" currentRawValue1: ");
  Serial.print(currentRawValue1);
  Serial.print(" currentRawValue2: ");
  Serial.print(currentRawValue2);
  Serial.print(" difference: ");
  Serial.println(int32_t(currentRawValue1)-int32_t(currentRawValue2));
}

void THERMOCOUPLE::exponentialMovingAverage()
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

float THERMOCOUPLE::linearRegressionLeastSquared_PID()
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
