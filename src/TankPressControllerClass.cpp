#include "TankPressControllerClass.h"
#include <Arduino.h>

TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent, float setVentFailsafePressure_Default, bool setIsSystemBang, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent}, tankVent{*setTankVent}, ventFailsafePressure_Default{setVentFailsafePressure_Default}, isSystemBang{setIsSystemBang}, nodeIDCheck{setNodeIDCheck}
{
    // Instantiation stuff?
}
TankPressController::TankPressController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPrimaryPressValve, Valve* setPressLineVent, Valve* setTankVent, float setTargetPcValue_Default, float setTankToChamberDp_Default, float setVentFailsafePressure_Default, float set_K_p_Default, float set_K_i_Default, float set_K_d_Default, float setControllerThreshold_Default, bool setIsSystemBang, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, primaryPressValve{*setPrimaryPressValve}, pressLineVent{*setPressLineVent}, tankVent{*setTankVent}, targetPcValue_Default{setTargetPcValue_Default}, tankToChamberDp_Default{setTankToChamberDp_Default}, ventFailsafePressure_Default{setVentFailsafePressure_Default}, K_p_Default{set_K_p_Default}, K_i_Default{set_K_i_Default}, K_d_Default{set_K_d_Default}, controllerThreshold_Default{setControllerThreshold_Default}, isSystemBang{setIsSystemBang}, nodeIDCheck{setNodeIDCheck}
{
    // Instantiate operational values from the default values given
    // Allows a reset to defaults after having changed settings via config messages
    targetPcValue = targetPcValue_Default;
    tankToChamberDp = tankToChamberDp_Default;
    //targetValue = targetValue_Default;
    targetValue = targetPcValue + tankToChamberDp;

    K_p = K_p_Default;
    K_i = K_i_Default;
    K_d = K_d_Default;
    ventFailsafePressure = ventFailsafePressure_Default;
    controllerThreshold = controllerThreshold_Default;
    valveMinimumDeenergizeTime = valveMinimumDeenergizeTime_Default;
    valveMinimumEnergizeTime = valveMinimumEnergizeTime_Default;
    
    // force K_i = 0 at instantiation - should move this now with new defaults setup
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
    
    if (bangSensorWeightedEMA >= ventFailsafePressure)
    {
        if (ventFailsafeFlag)
        {
        //tankVent.setState(ValveState::OpenCommanded);
        }
        ventFailsafeFlag = true;
        //Serial.print(bangSensor1EMA);
        //Serial.print(" : ");
        //Serial.println(ventFailsafePressure);
    }
    else ventFailsafeFlag = false;
    }
}

void TankPressController::stateOperations()
{
    //run the PID calculation each time state operations runs
    //timer use real timestep here?
    // Each time this is run, set controllerUpdate to true
    controllerUpdate = true;

    if (isSystemBang)
    {
        // Hold the reset integral calc bool true each cycle while in T minus to prevent windup
        if (currentAutosequenceTime <= 0)
        {
            resetIntegralCalcBool = true;
        }
    // Run PID math on BangBang Tank Controllers
    bangPIDoutput = PIDmath();
    }

    // Controller State switch case
    switch (state)
    {
    case TankPressControllerState::Passive:
        testPass = false;
        //set abortFlag false when going to passive;
        abortFlag = false;
        //don't do shit
        //if (priorState != TankPressControllerState::Passive)
        //{
        //primaryPressValve.setState(ValveState::CloseCommanded);
        //pressLineVent.setState(ValveState::CloseCommanded);
        //tankVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Slow;
        //}
        break;
    case TankPressControllerState::Standby:
        testPass = false;
        //set abortFlag false when going to passive;
        abortFlag = false;
        //don't do shit
        if (priorState != TankPressControllerState::Standby)
        {
        primaryPressValve.setState(ValveState::CloseCommanded);
        pressLineVent.setState(ValveState::CloseCommanded);
        tankVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Slow;
        }
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
        //set abortFlag false going into Vent to be able to vent out of an Abort from abortFlag
        abortFlag = false;
        if (priorState != TankPressControllerState::Vent)
        {
            //Serial.println("dis u? ");
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

/* // controller Valve object stateOperations
    primaryPressValve.controllerStateOperations();
    pressLineVent.controllerStateOperations();
    tankVent.controllerStateOperations(); */
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
    bangSensorWeightedEMA = bangSensor1EMA;
    e_p = targetValue - bangSensor1EMA;
    e_i = bangSensor1Integral;
    e_d = bangSensor1Derivative;
    }
    if (!trustBangSensor1 && trustBangSensor2)
    {
    bangSensorWeightedEMA = bangSensor2EMA;
    e_p = targetValue - bangSensor2EMA;
    e_i = bangSensor2Integral;
    e_d = bangSensor2Derivative;
    }
    if (trustBangSensor3 && !trustBangSensor1 && !trustBangSensor2)
    {
    bangSensorWeightedEMA = bangSensor3EMA;
    e_p = targetValue - bangSensor3EMA;
    e_i = bangSensor3Integral;
    e_d = bangSensor3Derivative;
    }
    if (!trustBangSensor1 && !trustBangSensor2 && !trustBangSensor3)
    {
        // if controller doesn't trust ANY tank sensor, even the sim, trigger an abort
        abortFlag = true;
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

void TankPressController::setPcTarget(float PcTargetIn)
{
    if (PcTargetIn <= 600 && PcTargetIn >= 200) 
    {
        // Only continue if the new target is different than current target
        if (targetPcValue != PcTargetIn)
        {
            resetIntegralCalcBool = true;
            targetPcValue = PcTargetIn;
            // set target point based on dP, make the math function later
            //targetValue = targetPcValue + tankToChamberDp;
            if (!isWaterFlowSetup)
            {
            targetValue = targetPcValue*1.25; //very crude dP approx
            }
            else 
            {
            targetValue = targetPcValue*0.25; //very crude dP approx
            }
        }
    }
}


void TankPressController::resetAll()
{
    ventFailsafePressure = ventFailsafePressure_Default;
    targetPcValue = targetPcValue_Default;
    tankToChamberDp = tankToChamberDp_Default;
    //targetValue = targetValue_Default;
    targetValue = targetPcValue + tankToChamberDp;

    K_p = K_d_Default;
    K_i = K_i_Default;
    K_d = K_d_Default;
    controllerThreshold = controllerThreshold_Default;
    valveMinimumDeenergizeTime = valveMinimumDeenergizeTime_Default;
    valveMinimumEnergizeTime = valveMinimumEnergizeTime_Default;
    
    // force K_i = 0 at instantiation - should move this now with new defaults setup
    K_i_run = K_i;  //stashes K_i value for later
    K_i = 0;

}