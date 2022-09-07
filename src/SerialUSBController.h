#ifndef SERIALUSBCONTROLLER_h
#define SERIALUSBCONTROLLER_h

#include <Arduino.h>
#include <array>
using std::array;
#include "ControlFunctionsPasaBang.h" //need to shift the include tree into generic control function includes probably but for now leave it just PasaBang

class SerialUSBController
{
    private:
    bool propStatusPrints = false;
    bool propCSVStreamPrints = false;

    public:
    //
    void setPropStatusPrints(bool printSetIn){propStatusPrints = printSetIn;}
    void setPropCSVStreamPrints(bool printSetIn){propCSVStreamPrints = printSetIn;}
    
    // Only at most one of these should be active at any given time
    // Functiom for Programming/Debugging Serial Print stream
    void propulsionNodeStatusPrints(VehicleState& currentVehicleState, VehicleState& priorVehicleState, MissionState& currentMissionState, MissionState& prionMissionState, Command& currentCommand, commandMSG& currentCommandMSG, configMSG& currentConfigMSG, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<SENSORBASE*, NUM_HPSENSORS>& HPsensorArray, const uint8_t& propulsionNodeIDIn);
    // Functiom for data logging/CSV Serial Print stream
    void propulsionNodeCSVStreamPrints(VehicleState& currentVehicleState, VehicleState& priorVehicleState, MissionState& currentMissionState, MissionState& prionMissionState, Command& currentCommand, commandMSG& currentCommandMSG, configMSG& currentConfigMSG, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const uint8_t& propulsionNodeIDIn);

};

#endif