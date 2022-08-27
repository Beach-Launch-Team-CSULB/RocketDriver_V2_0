#include "SerialUSBController.h"

void SerialUSBController::propulsionNodeStatusPrints(VehicleState& currentVehicleState, VehicleState& priorVehicleState, MissionState& currentMissionState, MissionState& prionMissionState, Command& currentCommand, commandMSG& currentCommandMSG, configMSG& currentConfigMSG, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn)
{
    // Only print if both this bool is true and CSV print bool is false
    if (propStatusPrints && !propCSVStreamPrints)
    {

  //Main Loop state and command print statements - for testing only - TEMPORARY BULLSHIT
  Serial.print("prop node ID : ");
  Serial.print(propulsionNodeIDIn);
  Serial.print(" currentVehicleState :");
  Serial.println(static_cast<uint8_t>(currentVehicleState));
  Serial.print(" currentCommand :");
  Serial.println(currentCommand);

//vectorBufferPrintout();

  Serial.print(" currentConfigMSG :");
  Serial.print(" targetID :");
  Serial.print(currentConfigMSG.TargetObjectID);
  Serial.print(" settingID:");
  Serial.print(currentConfigMSG.ObjectSettingID);
  Serial.print(" float:");
  Serial.print(currentConfigMSG.floatValue);
  Serial.print(" uint8:");
  Serial.print(currentConfigMSG.uint8Value);

  Serial.println();
  Serial.print(fluidSim.TimeDelta, 10);
  Serial.print(" : ");
  Serial.print(fluidSim.FuelTank.CurrPressure/6895, 5);
  Serial.print(" : ");
  Serial.print(fluidSim.LoxTank.CurrPressure/6895, 5);
  Serial.print(" : ");
  Serial.print(fluidSim.HiPressTank.CurrPressure/6895, 5);
  Serial.println(" fluid sim update ran");

    for(auto tankPressController : tankPressControllerArray)
    {
            Serial.print( ": TankControllerState: ");
            Serial.print(static_cast<uint8_t>(tankPressController->getState()));
            Serial.println(": ");
            Serial.print(static_cast<uint8_t>(tankPressController->getPrimaryPressValveState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(tankPressController->getPressLineVentState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(tankPressController->getTankVentState()));
/*             Serial.print(": bangTimer: ");
            Serial.print(tankPressController->bangtimer);
            Serial.print(": minDeEnergizeTime: ");
            Serial.print(tankPressController->valveMinimumDeenergizeTime);
            Serial.print(": minEnergizeTime: ");
            Serial.print(tankPressController->valveMinimumEnergizeTime);
            Serial.println(": "); */
            if (tankPressController->getIsBang())
            {
            Serial.print(": Target");
            Serial.print(tankPressController->getTargetValue(),5);
            Serial.print(": K_p");
            Serial.print(tankPressController->getKp(),5);
            Serial.print(": K_i");
            Serial.print(tankPressController->getKi(),5);
            Serial.print(": K_d");            
            Serial.print(tankPressController->getKd(),5);
            Serial.print(": e_p");
            Serial.print(tankPressController->getPfunc(),5);
            Serial.print(": e_i");
            Serial.print(tankPressController->getIfunc(),5);
            Serial.print(": e_d");            
            Serial.print(tankPressController->getDfunc(),5);
            Serial.print(": PID result");
            Serial.println(tankPressController->getPIDoutput(),5);
            }

    }    
    for(auto engineController : engineControllerArray)
    {
            Serial.print( ": EngineControllerState: ");
            Serial.print(static_cast<uint8_t>(engineController->getState()));
            Serial.println(": ");
            Serial.print(static_cast<uint8_t>(engineController->getPilotMVFuelValveState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(engineController->getPilotMVLoxValveState()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(engineController->getIgniter1State()));
            Serial.print(": ");
            Serial.print(static_cast<uint8_t>(engineController->getIgniter2State()));
            Serial.println(": ");

            for (auto i = engineController->throttleProgram.begin(); i != engineController->throttleProgram.end(); ++i)
            {
                Serial.print(" throttle program point: ");
                Serial.print(" time: ");
                Serial.print(i->autoSequenceTimeValue);
                Serial.print(" Pc: ");
                Serial.println(i->targetPcValue);
            }

    }
    
    for(auto valve : valveArray)
    {
            //Serial.print("ValveNodeID: ");
            //Serial.print(static_cast<uint8_t>(valve->getValveNodeID()));
            //Serial.print("ValveID: ");
            //Serial.print(static_cast<uint8_t>(valve->getValveID()));
        
        if (valve->getValveNodeID() == propulsionNodeIDIn)
        {
            Serial.print(" ValveID: ");
            Serial.print(static_cast<uint8_t>(valve->getValveID()));
            Serial.print( ": priorState: ");
            Serial.print(static_cast<uint8_t>(valve->getPriorState()));
            Serial.print( ": ValveState: ");
            Serial.print(static_cast<uint8_t>(valve->getState()));
            Serial.print(": ");
/*             Serial.print( ": ValveType: ");
            Serial.print(static_cast<uint8_t>(valve->getValveType()));
            Serial.print( ": HP Channel: ");
            Serial.print(valve->getHPChannel());
            Serial.print( ": PinDigital: ");
            Serial.print(valve->getPinDigital());
            Serial.print( ": PinPWM: ");
            Serial.print(valve->getPinPWM());
            Serial.print( ": PinADC: ");
            Serial.print(valve->getPinADC()); */
            Serial.println(": ");
        }
    }
    for(auto pyro : pyroArray)
    {
        
            //Serial.print("PyroNodeID: ");
            //Serial.print(static_cast<uint8_t>(pyro->getPyroNodeID()));
            //Serial.print("PyroID: ");
            //Serial.print(static_cast<uint8_t>(pyro->getPyroID()));
       if (pyro->getPyroNodeID() == propulsionNodeIDIn)
         {
            Serial.print(" PyroID:  ");
            Serial.print(static_cast<uint8_t>(pyro->getPyroID()));
            Serial.print( ": PyroState:  ");
            Serial.print(static_cast<uint8_t>(pyro->getState()));
/*             Serial.print( ": HP Channel: ");
            Serial.print(pyro->getHPChannel());
            Serial.print( ": PinDigital: ");
            Serial.print(pyro->getPinDigital());
            Serial.print( ": PinPWM: ");
            Serial.print(pyro->getPinPWM());
            Serial.print( ": PinADC: ");
            Serial.print(pyro->getPinADC()); */
            Serial.println(": ");
        }
    }

  
    for(auto sensor : sensorArray)
    {
        if (sensor->getSensorNodeID() == propulsionNodeIDIn)
        {
        //sensor->setState(SensorState::Slow);
         
            Serial.print("SensorID: ");
            Serial.print(static_cast<uint8_t>(sensor->getSensorID()));
            //Serial.print( ": new converted bool: ");
            //Serial.print( ": new raw bool: ");
            //Serial.print(sensor->getNewSensorValueCheckCAN());
            //Serial.print(sensor->getNewSensorConversionCheck());
            Serial.print( ": raw value: ");
            Serial.print(sensor->getCurrentRawValue());
            //Serial.print( ": timestamp S: ");
            //Serial.print(sensor->getTimestampSeconds());
            //Serial.print( ": timestamp uS: ");
            //Serial.print(sensor->getTimestampMicros());

            Serial.print( ": converted: ");
            Serial.print(static_cast<float>(sensor->getCurrentConvertedValue()));
            Serial.print( ": EMA: ");
            Serial.print(sensor->getEMAConvertedValue(),10);
            Serial.print( ": Integral Enabled: ");
            Serial.print(sensor->getEnableIntegralCalc());
            Serial.print( ": I: ");
            Serial.print(sensor->getIntegralSum(),10);
            //if (sensor->getEnableLinearRegressionCalc())
            //{
            Serial.print( ": LinReg Enabled: ");
            Serial.print(sensor->getEnableLinearRegressionCalc());
            Serial.print( ": D: ");
            Serial.print(sensor->getLinRegSlope(),10);
            //}
            
            Serial.println(": ");

        }
    
    }

  Serial.print("Current Autosequence Time: ");
  Serial.println(autoSequenceArray.at(0)->getCurrentCountdown());

    }
      //Serial.print("EEPROM Node ID Read :");
  //Serial.println(EEPROM.read(nodeIDAddress));

    //ALARASN& thisALARA = ALARASNmap[ALARAnodeID];
/*     Serial.print("prop system nodeID: ");
    Serial.println(thisALARA.propulsionSysNodeID);
    Serial.print("board rev: ");
    Serial.println(static_cast<uint8_t>(thisALARA.boardRev)); */
}


void SerialUSBController::propulsionNodeCSVStreamPrints(VehicleState& currentVehicleState, VehicleState& priorVehicleState, MissionState& currentMissionState, MissionState& prionMissionState, Command& currentCommand, commandMSG& currentCommandMSG, configMSG& currentConfigMSG, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn)
{
    // Prints active if bool is true
    if (propCSVStreamPrints)
    {
    
    // Fluid Sim
    Serial.print(fluidSim.TimeDelta, 5);
    Serial.print(", ");   // comma delimeter
    Serial.print(fluidSim.FuelTank.CurrPressure/6895, 5);
    Serial.print(", ");   // comma delimeter
    Serial.print(fluidSim.LoxTank.CurrPressure/6895, 5);
    Serial.print(", ");   // comma delimeter
    Serial.print(fluidSim.HiPressTank.CurrPressure/6895, 5);
    Serial.print(", ");   // comma delimeter
    // Tank Controllers
    for(auto tankPressController : tankPressControllerArray)
    {
    Serial.print(static_cast<uint8_t>(tankPressController->getState()));
    Serial.print(", ");   // comma delimeter

    if (tankPressController->getIsBang())
    {
    Serial.print(tankPressController->getTargetValue(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getKp(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getKi(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getKd(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getPfunc(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getIfunc(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getDfunc(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(tankPressController->getPIDoutput(),5);
    Serial.print(", ");   // comma delimeter
    }

    }
    // Sensors
    for(auto sensor : sensorArray)
    {
    if (sensor->getSensorNodeID() == propulsionNodeIDIn)
    {
    Serial.print(sensor->getCurrentRawValue());
    Serial.print(", ");   // comma delimeter
    //Serial.print( ": timestamp S: ");
    //Serial.print(sensor->getTimestampSeconds());
    //Serial.print( ": timestamp uS: ");
    //Serial.print(sensor->getTimestampMicros());

    Serial.print(static_cast<float>(sensor->getCurrentConvertedValue()));
    Serial.print(", ");   // comma delimeter
    Serial.print(sensor->getEMAConvertedValue(),5);
    Serial.print(", ");   // comma delimeter
    Serial.print(sensor->getIntegralSum(),5);
    if (sensor->getEnableLinearRegressionCalc())
    {
    Serial.print(", ");   // comma delimeter
    Serial.print(sensor->getLinRegSlope(),5);
    Serial.print(", ");   // comma delimeter
    }
    else 
    {
    Serial.print(", ");   // comma delimeter
    Serial.print("No Lin Reg Slope");   // comma delimeter
    Serial.print(", ");   // comma delimeter
    }

    }
    
    }

    // End
    Serial.println();  //end line change
    }
    



}