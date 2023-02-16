#ifndef CONTROLFUNCTIONS_H
#define CONTROLFUNCTIONS_H
#pragma once

#include "StateList.h"
#include "ControlCommands.h"
#include "ValveClass.h"
#include "PyroClass.h"
#include "AutoSequenceClass.h"
#include "EXTSensorClass.h"
#include "ALARAHPSensorClass.h"
#include "TankPressControllerClass.h"
#include "EngineControllerClass.h"
#include "configurationSet.h"
#include "ALARABoardControllerClass.h"
#include "fluidSystemSimulation.h"
#include "ALARABoardControllerClass.h"
#include <array>


#ifdef RENEGADESF
// Renegade SF Stand - somewhat clumsy to define again, not sure a cleaner way to integrate
#define NUM_VALVES 10
#define NUM_PYROS 2
#define NUM_AUTOSEQUENCES 1
#define NUM_SENSORS 23
#define NUM_HPSENSORS 20
#define NUMINTERNAL_SENSORS 13      // On EACH ALARA
#define NUM_TANKPRESSCONTROLLERS 3
#define NUM_ENGINECONTROLLERS 1

// checks the state that was set at start-up and issues the appropriate command as current command
void startupStateCheck(const VehicleState& currentState, Command& currentCommand);
// command processing to set vehicle State
void commandExecute(VehicleState& currentState, VehicleState& priorState, MissionState& currentMissionState, MissionState prionMissionState, Command& currentCommand, bool& newCommand, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);
// function for checking if any controller has flagged for an abort
void controllerAbortCheck(VehicleState& currentState, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);
// state machine for operating all devices on the vehicle
void vehicleStateMachine(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, ALARABoardController& boardController, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim, bool &HaltFlag, bool& outputOverride);
// state machine for the mission state (launch, ascent, apogee, descent et cetera)
void missionStateMachine(VehicleState& currentState, VehicleState& priorState, MissionState& currentMissionState, MissionState prionMissionState, ALARABoardController& boardController, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, bool& staticTestIn, bool &HaltFlag);

// ----- Controller Functions -----
void controllerDeviceSync(VehicleState& currentState, VehicleState& priorState, Command& currentCommand, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim, bool &HaltFlag);

void controllerSensorSetup(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);

void controllerDataSync(const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray);

void configMSGread(configMSG& currentConfigMSG, bool& NewConfigMessage, const std::array<Valve*, NUM_VALVES>& valveArray, const std::array<Pyro*, NUM_PYROS>& pyroArray, const std::array<SENSORBASE*, NUM_SENSORS>& sensorArray, const std::array<AutoSequence*, NUM_AUTOSEQUENCES>& autoSequenceArray, const std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS>& tankPressControllerArray, const std::array<EngineController*, NUM_ENGINECONTROLLERS>& engineControllerArray, FluidSystemSimulation& fluidSim);


#endif

#endif