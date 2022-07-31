#ifndef CONTROLFUNCTIONSPASABANG_H
#define CONTROLFUNCTIONSPASABANG_H

#include "StateList.h"
#include "ControlCommands.h"
#include "ValveClass.h"
#include "PyroClass.h"
#include "AutoSequenceClass.h"
#include "EXTSensorClass.h"
#include "TankPressControllerClass.h"
#include "EngineControllerClass.h"
#include "configurationSet.h"
#include <array>

// Pasafire Bang - somewhat clumsy to define again, not sure a cleaner way to integrate
#define NUM_VALVES 7
#define NUM_PYROS 2
#define NUM_AUTOSEQUENCES 1
#define NUM_SENSORS 14
#define NUMINTERNAL_SENSORS 13      // On EACH ALARA
#define NUM_TANKPRESSCONTROLLERS 3
#define NUM_ENGINECONTROLLERS 1

// checks the state that was set at start-up and issues the appropriate command as current command
void startupStateCheck(const VehicleState& currentState, Command& currentCommand);
// state machine for operating all devices on the vehicle
void vehicleStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool &HaltFlag);
// state machine for the mission state (launch, ascent, apogee, descent et cetera)
void missionStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, bool &HaltFlag);

// ----- Controller Functions -----
void controllerDeviceSync(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, bool &HaltFlag);

void controllerSensorSetup(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);

void controllerDataSync(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);

void configMSGread(configMSG& currentConfigMSG, bool& NewConfigMessage, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);

#endif