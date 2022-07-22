#include "EngineControllerClass.h"
#include <Arduino.h>

EngineController::EngineController(uint32_t setControllerID, uint8_t setControllerNodeID, int64_t setFuelMVAutosequenceActuation, int64_t setLoxMVAutosequenceActuation, int64_t setIgniter1Actuation, int64_t setIgniter2Actuation, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, fuelMVAutosequenceActuation{setFuelMVAutosequenceActuation}, loxMVAutosequenceActuation{setLoxMVAutosequenceActuation}, igniter1Actuation{setIgniter1Actuation}, igniter2Actuation{setIgniter2Actuation}, nodeIDCheck{setNodeIDCheck}
{
    // Instantiation stuff?
}

void EngineController::begin()
{
    if (nodeIDCheck)
    {
        // setup stuff?
    }
}

void EngineController::resetTimer()
{
    timer = 0;
}

void EngineController::stateOperations()
{
    switch (state)
    {
    case EngineControllerState::Passive:
        //don't do shit
        pilotMVFuelValveState = ValveState::CloseCommanded;
        pilotMVLoxValveState = ValveState::CloseCommanded;
        pneumaticVentState = ValveState::CloseCommanded;
        sensorState = SensorState::Slow;
        igniter1State = PyroState::OffCommanded;
        igniter2State = PyroState::OffCommanded;
        break;
    case EngineControllerState::Armed:
        // Arming turns sensor read rates up to operational levels before opening valves
        if (priorState != EngineControllerState::Armed)
        {
        pilotMVFuelValveState = ValveState::CloseCommanded;
        pilotMVLoxValveState = ValveState::CloseCommanded;
        pneumaticVentState = ValveState::CloseCommanded;
        sensorState = SensorState::Fast;
        igniter1State = PyroState::OffCommanded;
        igniter2State = PyroState::OffCommanded;
        }
        break;
    case EngineControllerState::FiringAutosequence:
        // I DO need this case to run every stateOperations cycle to check on sequence timing, or I need to move logic somewhere else
        //Fuel MV autosequence check
        if (currentAutosequenceTime >= fuelMVAutosequenceActuation) {pilotMVFuelValveState = ValveState::OpenCommanded;}
        else {pilotMVFuelValveState = ValveState::FireCommanded;}
        //Lox MV autosequence check
        if (currentAutosequenceTime >= loxMVAutosequenceActuation) {pilotMVLoxValveState = ValveState::OpenCommanded;}
        else {pilotMVLoxValveState = ValveState::FireCommanded;}
        //Engine Igniter1 autosequence check
        if (currentAutosequenceTime >= igniter1Actuation) {igniter1State = PyroState::OnCommanded;}
        else {igniter1State = PyroState::FireCommanded;}
        //Engine Igniter2 autosequence check
        if (currentAutosequenceTime >= igniter2Actuation) {igniter2State = PyroState::OnCommanded;}
        else {igniter2State = PyroState::FireCommanded;}
        // devices not conditional within autosequence below
        pneumaticVentState = ValveState::CloseCommanded;
        sensorState = SensorState::Fast;
        break;
    case EngineControllerState::Shutdown:
        if (priorState != EngineControllerState::Shutdown)
        {
        sensorState = SensorState::Fast;
        pilotMVFuelValveState = ValveState::CloseCommanded;
        pilotMVLoxValveState = ValveState::CloseCommanded;
        pneumaticVentState = ValveState::CloseCommanded;
        sensorState = SensorState::Fast;
        igniter1State = PyroState::OffCommanded;
        igniter2State = PyroState::OffCommanded;
        }
        break;
    case EngineControllerState::TestPassthrough:
        sensorState = SensorState::Slow;
        // How to handle test and offnominal pass through? figure out after I've got valveArray pointers passed functioning
        break;
    default:
        break;
    }
}
