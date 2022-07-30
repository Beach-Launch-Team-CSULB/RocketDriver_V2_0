#include "TankPressControllerClass.h"
#include <Arduino.h>

TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent, float setTargetValue, float setVentFailsafePressure, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent}, tankVent{*setTankVent}, targetValue{setTargetValue}, ventFailsafePressure{setVentFailsafePressure}, nodeIDCheck{setNodeIDCheck}
{
    // Instantiation stuff?
}
TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent, float setTargetValue, float setVentFailsafePressure, float set_K_p, float set_K_i, float set_K_d, float setControllerThreshold, bool setIsSystemBang, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent}, tankVent{*setTankVent}, targetValue{setTargetValue}, ventFailsafePressure{setVentFailsafePressure}, K_p{set_K_p}, K_i{set_K_i}, K_d{set_K_d}, controllerThreshold{setControllerThreshold}, isSystemBang{setIsSystemBang}, nodeIDCheck{setNodeIDCheck}
{
    // force K_i = 0 at instantiation
    K_i_run = K_i;  //stashes K_i value for later
    K_i = 0;
    //TankPressController::setK_i(0);
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

void TankPressController::ventPressureCheck()
{
    if (isSystemBang)   // lazy check to not run on high press currently, would break on dome reg system for tanks
    {
    
    if (bangSensor1EMA >= ventFailsafePressure)
    {
        //Serial.print(bangSensor1EMA);
        //Serial.print(" : ");
        //Serial.println(ventFailsafePressure);
        tankVent.setState(ValveState::OpenCommanded);
    }
    }
}

void TankPressController::stateOperations()
{
    //run the PID calculation each time state operations runs
    if (isSystemBang)
    {
    bangPIDoutput = PIDmath();
    }

    // Controller State switch case
    switch (state)
    {
    case TankPressControllerState::Passive:
        testPass = false;
        //don't do shit
        primaryPressValve.setState(ValveState::CloseCommanded);
        pressLineVent.setState(ValveState::CloseCommanded);
        tankVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Slow;
        break;
    case TankPressControllerState::RegPressActive:
        testPass = false;
        //do shit
        if (priorState != TankPressControllerState::RegPressActive)
        {
        sensorState = SensorState::Fast;
        primaryPressValve.setState(ValveState::OpenCommanded);
        pressLineVent.setState(ValveState::CloseCommanded);
        tankVent.setState(ValveState::CloseCommanded);
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        }
        break;
    case TankPressControllerState::Armed:
        testPass = false;
        if (priorState != TankPressControllerState::Armed)
        {
        // Arming turns sensor read rates up to operational levels before opening valves
        sensorState = SensorState::Fast;
        primaryPressValve.setState(ValveState::CloseCommanded);
        pressLineVent.setState(ValveState::CloseCommanded);
        tankVent.setState(ValveState::CloseCommanded);
        }
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    case TankPressControllerState::Vent:
        testPass = false;
        if (priorState != TankPressControllerState::Vent)
        {
        sensorState = SensorState::Fast;
        primaryPressValve.setState(ValveState::CloseCommanded);
        pressLineVent.setState(ValveState::OpenCommanded);
        tankVent.setState(ValveState::OpenCommanded);
        }
        break;
    case TankPressControllerState::Abort:
        testPass = false;
        if (priorState != TankPressControllerState::Abort)
        {
        sensorState = SensorState::Fast;
        primaryPressValve.setState(ValveState::CloseCommanded);
        pressLineVent.setState(ValveState::CloseCommanded);
        tankVent.setState(ValveState::CloseCommanded);
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        }
        break;
    case TankPressControllerState::HiPressPassthroughVent:  //uhh what the fuck was this, the valve states are ???
        testPass = false;
        if (priorState != TankPressControllerState::HiPressPassthroughVent)
        {
        sensorState = SensorState::Fast;
        primaryPressValve.setState(ValveState::CloseCommanded);
        pressLineVent.setState(ValveState::OpenCommanded);
        tankVent.setState(ValveState::CloseCommanded);
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        }
        break;
    case TankPressControllerState::TestPassthrough:
        sensorState = SensorState::Slow;
        //
        testPass = true;
        break;
    case TankPressControllerState::OffNominalPassthrough:
        //
        testPass = true;
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    case TankPressControllerState::AutosequenceCommanded:
        testPass = false;
        // If specific press routine is on autosequence, include a state switch on timer in here
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
        break;
    case TankPressControllerState::BangBangActive:
        testPass = false;
        //minimum bang time lockouts, once they are up the valves go to plain Open/Closed states which unlocks them to be commanded again
        if (primaryPressValve.getState() == ValveState::BangingOpen || primaryPressValve.getState() == ValveState::BangOpenProcess)
        {
            if (bangtimer >= valveMinimumEnergizeTime)    // X ms opening/closing time
            {
            primaryPressValve.setState(ValveState::Open);
            }
        }
        if (primaryPressValve.getState() == ValveState::BangingClosed)
        {
            if (bangtimer >= valveMinimumDeenergizeTime)    // X ms opening/closing time
            {
            primaryPressValve.setState(ValveState::Closed);
            }
        }
        // Update ValveState if Open/Closed based on PID controller output
        if (bangPIDoutput > (controllerThreshold))
        {
            //open valve
            if (primaryPressValve.getState() == ValveState::Closed)
            {
            primaryPressValve.setState(ValveState::BangOpenCommanded);
            bangtimer = 0;
            }
            
        }
        if (bangPIDoutput < ((-1)*controllerThreshold))
        {
            //close valve
            if (primaryPressValve.getState() == ValveState::Open)
            {
            primaryPressValve.setState(ValveState::BangCloseCommanded);
            bangtimer = 0;
            }
        }
        sensorState = SensorState::Fast;
        ventPressureCheck();    //overpress failsafe, opens vent above failsafe pressure. Does not change controller state, only does pressure relief
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
        pressLineVent.setState(ValveState::OpenCommanded);
    }

}
else
{
    pressLineVent.setState(pressLineVentStateBang1);
}

// controller Valve object stateOperations
    primaryPressValve.controllerStateOperations();
    pressLineVent.controllerStateOperations();
    tankVent.controllerStateOperations();
}

void TankPressController::setPIDSensorInput1(float proportionalValue, float integralValue, float derivativeValue)
{
    bangSensor1EMA = proportionalValue;
    bangSensor1Integral = integralValue;
    bangSensor1Derivative = derivativeValue;
}
void TankPressController::setPIDSensorInput2(float proportionalValue, float integralValue, float derivativeValue)
{
    bangSensor2EMA = proportionalValue;
    bangSensor2Integral = integralValue;
    bangSensor2Derivative = derivativeValue;
}
void TankPressController::setPIDSensorInput3(float proportionalValue, float integralValue, float derivativeValue)
{
    bangSensor3EMA = proportionalValue;
    bangSensor3Integral = integralValue;
    bangSensor3Derivative = derivativeValue;
/*     Serial.print("bang3 ins: ");
    Serial.print(bangSensor3EMA);
    Serial.print(" : ");
    Serial.print(bangSensor3Integral);
    Serial.print(" : ");
    Serial.println(bangSensor3Derivative); */

}

void TankPressController::PIDinputSetting()
{
    if (trustBangSensor1)
    {
    e_p = targetValue - bangSensor1EMA;
    e_i = bangSensor1Integral;
    e_d = bangSensor1Derivative;
    }
    if (!trustBangSensor1 && trustBangSensor2)
    {
    e_p = targetValue - bangSensor2EMA;
    e_i = bangSensor2Integral;
    e_d = bangSensor2Derivative;
    }
    if (trustBangSensor3 && !trustBangSensor1 && !trustBangSensor2)
    {
    e_p = targetValue - bangSensor3EMA;
    e_i = bangSensor3Integral;
    e_d = bangSensor3Derivative;
    }
}

//float PIDmath(float inputArrayPID[], float controllerSetPoint, float timeStepPIDMath, float integrationSteps, float errorThreshold, float K_p, float K_i, float K_d)
float TankPressController::PIDmath()
{
    PIDinputSetting();
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
