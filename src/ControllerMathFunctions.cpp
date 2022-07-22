#include "ControllerMathFunctions.h"
#include <Arduino.h>
#include <math.h>


int rollingAve_n = 0;
int arrayWrapSizeRollingAve = 0;
int arrayMostRecentPositionRollingAve = 0;
int sizeInputArrayRollingAve = 0;
int arrayIndexFirstValueRollingAve = 0;
float rollingAverageReturn = 0;
int rollingAveSamples = 0;
float proportionalRollingAverage(float inputArrayRollingAve[], int rollingAveSamples)
{
  //rollingAveSamples = 0;
  sizeInputArrayRollingAve = 0;
  arrayMostRecentPositionRollingAve = 0;
  arrayIndexFirstValueRollingAve = 0;
  arrayIndexFirstValueRollingAve = static_cast<int>(inputArrayRollingAve[0]+0.5);
  arrayMostRecentPositionRollingAve = static_cast<int>(inputArrayRollingAve[1]+0.5);
  sizeInputArrayRollingAve = static_cast<int>(inputArrayRollingAve[2]+0.5);

  rollingAverageReturn = 0;
  rollingAve_n = 0;
  arrayWrapSizeRollingAve = 0;
  // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
  if (sizeInputArrayRollingAve < rollingAveSamples)
  {
    rollingAve_n = sizeInputArrayRollingAve;
  }
  else 
  {
    rollingAve_n = rollingAveSamples;
  }
/*       Serial.println("n from inside func BEFORE wrap math");
      Serial.println(rollingAve_n);
      Serial.println("rollingAve samples BEFORE wrap math");
      Serial.println(rollingAveSamples);
 */      
  //rollingAve_n = 4;
  // determine the overwrap value, if any
  arrayWrapSizeRollingAve = rollingAve_n - ((arrayMostRecentPositionRollingAve) - (arrayIndexFirstValueRollingAve) + 1);
//arrayWrapSizeRollingAve = 0;
/*       Serial.println("size input array rolling ave");
      Serial.println(sizeInputArrayRollingAve);
      Serial.println("array most recent index inside func");
      Serial.println(arrayMostRecentPositionRollingAve);
      Serial.println("n from inside func");
      Serial.println(rollingAve_n);
      Serial.println("array wrap side from inside func");
      Serial.println(arrayWrapSizeRollingAve);
 */      
  if (arrayWrapSizeRollingAve > 0) //only true if there are enough array values to use to wrap the end of the array
    {
      //for (int i = arrayMostRecentPositionRollingAve; i < (sizeInputArrayRollingAve + 2); i++)
      for (int i = arrayMostRecentPositionRollingAve; i > (arrayIndexFirstValueRollingAve - 1); i--)
      {
        rollingAverageReturn = rollingAverageReturn + ((inputArrayRollingAve[i]));
/*         Serial.print("rollingAverageValue: ");
        Serial.println(rollingAverageReturn);
 */
      }
      //for (int i = 2; i < (arrayWrapSizeRollingAve + 1); i++)
      for (int i = (sizeInputArrayRollingAve + arrayIndexFirstValueRollingAve - 1); i > (sizeInputArrayRollingAve + arrayIndexFirstValueRollingAve - 1 - arrayWrapSizeRollingAve); i--)
      {
        rollingAverageReturn = rollingAverageReturn + ((inputArrayRollingAve[i]));
/*         Serial.print("rollingAverageValue: ");
        Serial.println(rollingAverageReturn);
 */        
      }
    }
  else
  {
    for (int i = arrayMostRecentPositionRollingAve; i > (arrayMostRecentPositionRollingAve - rollingAve_n); i--)
    {
      rollingAverageReturn = rollingAverageReturn + ((inputArrayRollingAve[i]));
/*         Serial.print("rollingAverageValue: ");
        Serial.println(rollingAverageReturn);
 */        
    }
  }
  rollingAverageReturn = rollingAverageReturn/rollingAve_n;
  return rollingAverageReturn;
}

  int arrayIndexFirstValueLinReg = 0;
  int arrayWrapSizeLinReg = 0;
  int arrayMostRecentPositionLinReg = 0;
  int regression_n = 0;
  //float regression_PIDOutput = 0;
  int sizeInputArrayLinReg = 0;

float linearRegressionLeastSquared_PID(float inputArrayLinReg[], int regressionSamples, float timeStep)
{
  // Version of linear regression simplified for finding the recent slope for a PID controller
  // assumes fixed time steps, time is X, controller variable Y
  // !!!!! - Function is built to expect arrays in format of:
  // !!!!! - index[0] = first index with a value entry
  // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
  // !!!!! - index[2] = size of value entries
  // Not sure if I've updated the math to use the first value in the numerical part instead of just manually subtracting to make it work for when it's 3
  arrayIndexFirstValueLinReg = static_cast<int>(inputArrayLinReg[0]+0.5);
  arrayMostRecentPositionLinReg = static_cast<int>(inputArrayLinReg[1]+0.5);
  sizeInputArrayLinReg = static_cast<int>(inputArrayLinReg[2]+0.5);
  
    // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
  if (sizeInputArrayLinReg < regressionSamples)
  {
    regression_n = sizeInputArrayLinReg;
  }
  else regression_n = regressionSamples;
  // determine the overwrap value, if any
  //arrayWrapSizeLinReg =  (-1) * ((arrayMostRecentPositionLinReg - regression_n) - 1); //old array method
  arrayWrapSizeLinReg =  regression_n - ((arrayMostRecentPositionLinReg) - (arrayIndexFirstValueLinReg) + 1);

  float sumX = 0;
  float sumY = 0;
  float sumXX = 0;
  float sumXY = 0;
  float denLeastSquare = 0;
  float a0LeastSquare = 0;
  float a1LeastSquare = 0;
  // calculate the sum terms;
  // 
    //Serial.print("First for loop: ");
    //Serial.println(arrayWrapSizeLinReg);

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
      sumY = sumY + inputArrayLinReg[i];
      sumXY = sumXY + (inputArrayLinReg[i] * ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
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
      sumY = sumY + inputArrayLinReg[i];
      sumXY = sumXY + (inputArrayLinReg[i] * ((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 3)*timeStep));
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
      sumY = sumY + inputArrayLinReg[i];
      sumXY = sumXY + (inputArrayLinReg[i] * ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
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
  Serial.println(sumXY,8);
 */
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


float currentInputValue = 0;
float previousInputValue = 0;
int arrayIndexFirstValueAccumI = 0;
int arrayMostRecentPositionInsertAccumI = 0;
int sizeInputArrayInsert1AccumI = 0;
float accumIfuncOutput = 0;
float timeStepAccumI = 0;
float accumulatedI_PID_float(float inputArrayAccumI[], float timeStepAccumI, float sumZeroPointAccumI = 0)
{
  arrayIndexFirstValueAccumI = static_cast<int>(inputArrayAccumI[0]+0.5);
  arrayMostRecentPositionInsertAccumI = static_cast<int>(inputArrayAccumI[1]+0.5);
  sizeInputArrayInsert1AccumI = static_cast<int>(inputArrayAccumI[2]+0.5);

//why the fuck won't it use input timestep???
//timeStepAccumI = 1/600;

// Current and previous value lookup from rolling input array
  if (arrayMostRecentPositionInsertAccumI == (arrayIndexFirstValueAccumI)) // special case for being at the START of the array
  {
    currentInputValue = inputArrayAccumI[arrayMostRecentPositionInsertAccumI];
    previousInputValue = inputArrayAccumI[sizeInputArrayInsert1AccumI + arrayIndexFirstValueAccumI - 1];
  }
  else
  {
    currentInputValue = inputArrayAccumI[arrayMostRecentPositionInsertAccumI];
    previousInputValue = inputArrayAccumI[arrayMostRecentPositionInsertAccumI - 1];
  }
// trapazoid method for area under the curve using current and previous values as the end points
/* Serial.print("currentInputValue");
Serial.println(currentInputValue);
Serial.print("previousInputValue");
Serial.println(previousInputValue);
Serial.print("accumIfuncOutput");
Serial.println(accumIfuncOutput,10);
Serial.print("timeStepAccumI");
Serial.println(timeStepAccumI,10); */

  //accumIfuncOutput = timeStepAccumI * (((currentInputValue - sumZeroPointAccumI) + (previousInputValue - sumZeroPointAccumI))/2);
  accumIfuncOutput = (((currentInputValue - sumZeroPointAccumI) + (previousInputValue - sumZeroPointAccumI))/2);
  return accumIfuncOutput;
}
