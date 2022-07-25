#include "TankPressControllerClass.h"
#include <Arduino.h>

TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, uint32_t setTargetValue, bool setIsSystemBang, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, targetValue{setTargetValue}, isSystemBang{setIsSystemBang}, nodeIDCheck{setNodeIDCheck}
{
    // Instantiation stuff?
}

void TankPressController::begin()
{
    if (nodeIDCheck)
    {
        // setup stuff?
    }
}

void TankPressController::resetTimer()
{
    timer = 0;
}

void TankPressController::stateOperations()
{
    //run the PID calculation each time state operations runs
    bangPIDoutput = PIDmath();
    // Controller State switch case
    switch (state)
    {
    case TankPressControllerState::Passive:
        testPass = false;
        //don't do shit
        primaryPressValveState = ValveState::CloseCommanded;
        pressLineVentState = ValveState::CloseCommanded;
        tankVentState = ValveState::CloseCommanded;
        sensorState = SensorState::Slow;
        break;
    case TankPressControllerState::DomePressActive:
        testPass = false;
        //do shit
        if (priorState != TankPressControllerState::DomePressActive)
        {
        sensorState = SensorState::Fast;
        primaryPressValveState = ValveState::OpenCommanded;
        pressLineVentState = ValveState::CloseCommanded;
        tankVentState = ValveState::CloseCommanded;
        }
        break;
    case TankPressControllerState::Armed:
        testPass = false;
        // Arming turns sensor read rates up to operational levels before opening valves
        sensorState = SensorState::Fast;
        primaryPressValveState = ValveState::CloseCommanded;
        pressLineVentState = ValveState::CloseCommanded;
        tankVentState = ValveState::CloseCommanded;
        break;
    case TankPressControllerState::Vent:
        testPass = false;
        if (priorState != TankPressControllerState::Vent)
        {
        sensorState = SensorState::Fast;
        primaryPressValveState = ValveState::CloseCommanded;
        pressLineVentState = ValveState::OpenCommanded;
        tankVentState = ValveState::OpenCommanded;
        }
        break;
    case TankPressControllerState::HiPressPassthroughVent:
        testPass = false;
        if (priorState != TankPressControllerState::HiPressPassthroughVent)
        {
        sensorState = SensorState::Fast;
        primaryPressValveState = ValveState::OpenCommanded;
        pressLineVentState = ValveState::CloseCommanded;
        tankVentState = ValveState::OpenCommanded;
        }
        break;
    case TankPressControllerState::TestPassthrough:
        testPass = false;
        sensorState = SensorState::Slow;
        //
        testPass = true;
        break;
    case TankPressControllerState::AutosequenceCommanded:
        testPass = false;
        // If specific press routine is on autosequence, include a state switch on timer in here
        break;
    case TankPressControllerState::BangBangActive:
        testPass = false;
        //minimum bang time lockouts, once they are up the valves go to plain Open/Closed states which unlocks them to be commanded again
        if (primaryPressValveState == ValveState::BangingOpen)
        {
            if (bangtimer >= valveMinimumEnergizeTime)    // X ms opening/closing time
            {
            primaryPressValveState = ValveState::Open;
            }
        }
        if (primaryPressValveState == ValveState::BangingClosed)
        {
            if (bangtimer >= valveMinimumDeenergizeTime)    // X ms opening/closing time
            {
            primaryPressValveState = ValveState::Closed;
            }
        }
        // Update ValveState if Open/Closed based on PID controller output
        if (bangPIDoutput > (controllerThreshold))
        {
            //open valve
            if (primaryPressValveState == ValveState::Closed)
            {
            primaryPressValveState = ValveState::BangOpenCommanded;
            bangtimer = 0;
            }
            
        }
        if (bangPIDoutput < ((-1)*controllerThreshold))
        {
            //close valve
            if (primaryPressValveState == ValveState::Open)
            {
            primaryPressValveState = ValveState::BangCloseCommanded;
            bangtimer = 0;
            }
        }

        break;
    default:
        break;
    }

//External bangbang vent line logic
if (pressLineVentStateBang1 != pressLineVentStateBang2)
{
    //logic for when they don't agree - I should explicitly define all possible states but that's tomorrow me's problem
    //Most important one, allows the vent line opened if either bang tank controller commands it
    if (pressLineVentStateBang1 == ValveState::OpenCommanded || pressLineVentStateBang2 == ValveState::OpenCommanded)
    {
        pressLineVentState = ValveState::OpenCommanded;
    }
    

}
else
{
    pressLineVentState = pressLineVentStateBang1;
}

}

void TankPressController::setPIDSensorInputs(float proportionalValue, float integralValue, float derivativeValue)
{
    e_p = targetValue - proportionalValue;
    e_i = integralValue;
    e_d = derivativeValue;
}

//float PIDmath(float inputArrayPID[], float controllerSetPoint, float timeStepPIDMath, float integrationSteps, float errorThreshold, float K_p, float K_i, float K_d)
float TankPressController::PIDmath()
{
  //timeStepPIDMath = 1;
  funcOutput = 0;
  p_rollingAve = 0;
  P_p = 0;
  P_i = 0;
  P_d = 0;
/*   e_p = 0;
  //e_i = 0;
  e_d = 0; */
  //arrayMostRecentPositionPID = static_cast<int>(inputArrayPID[1]+0.5);

  // PID function calculations - new integral differenced int style
/*   e_p = controllerSetPoint - p_rollingAve;    // proportional offset calculation
  e_i += accumulatedI_PID_float(inputArrayPID, timeStepPIDMath, controllerSetPoint)* timeStepPIDMath;
  //e_p = controllerSetPoint - inputArrayPID[arrayMostRecentPositionPID];    // proportional offset calculation
  e_d = linearRegressionLeastSquared_PID(inputArrayPID, 5, timeStepPIDMath);    // derivative function calculation */

  //
  P_p = K_p*(e_p);
  P_i = K_i*(e_i);
  P_d = K_d*(e_d * controllerTimeStep);

if (isnan(P_p))
{
  P_p = 0;
}
if (isnan(P_i))
{
  P_i = 0;
}
if (isnan(P_d))
{
  P_d = 0;
}

  funcOutput = (P_p) - (P_i) - (P_d); //still not 100% sure on signs, particularly the d


  // normalizes units to be in PSI
  //funcOutput = (K_p*(e_p)) - (K_i*(e_i/integrationSteps)) - (K_d*(e_d * timeStep)); //still not 100% sure on signs, particularly the d

  if (PIDmathPrintFlag)
  {
    Serial.println("insidePID: ");
    Serial.print(K_p);
    Serial.print(" : ");
    Serial.print(e_p);
    Serial.print(" : ");
    Serial.println(K_p*(e_p));
    Serial.print(K_i);
    Serial.print(" : ");
    Serial.print(e_i);
    Serial.print(" : ");
    Serial.println(K_i*(e_i));
    Serial.print(K_d);  
    Serial.print(" : ");
    Serial.print(e_d);  
    Serial.print(" : ");
    Serial.println(K_d*(e_d * timeStepPIDMath));  
    Serial.println(funcOutput);
  }
    //Serial.print("insidePID p_rollingAve,");
    //Serial.print(p_rollingAve);
    //Serial.println(",");

  return funcOutput;
}
