#include "EngineControllerClass.h"
#include <Arduino.h>

EngineController::EngineController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPilotMVFuelValve, Valve* setPilotMVLoxValve, Valve* setPneumaticVent, Pyro* setIgniter1, Pyro* setIgniter2, int64_t setFuelMVAutosequenceActuation, int64_t setLoxMVAutosequenceActuation, int64_t setIgniter1Actuation, int64_t setIgniter2Actuation, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, pilotMVFuelValve{*setPilotMVFuelValve}, pilotMVLoxValve{*setPilotMVLoxValve}, pneumaticVent{*setPneumaticVent}, igniter1{*setIgniter1}, igniter2{*setIgniter2}, fuelMVAutosequenceActuation{setFuelMVAutosequenceActuation}, loxMVAutosequenceActuation{setLoxMVAutosequenceActuation}, igniter1Actuation{setIgniter1Actuation}, igniter2Actuation{setIgniter2Actuation}, nodeIDCheck{setNodeIDCheck}
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
        testPass = false;
        //don't do shit
        pilotMVFuelValve.setState(ValveState::CloseCommanded);
        pilotMVLoxValve.setState(ValveState::CloseCommanded);
        pneumaticVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Slow;
        igniter1.setState(PyroState::OffCommanded);
        igniter2.setState(PyroState::OffCommanded);
        break;
    case EngineControllerState::Armed:
        testPass = false;
        // Arming turns sensor read rates up to operational levels before opening valves
        if (priorState != EngineControllerState::Armed)
        {
        pilotMVFuelValve.setState(ValveState::CloseCommanded);
        pilotMVLoxValve.setState(ValveState::CloseCommanded);
        pneumaticVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Fast;
        igniter1.setState(PyroState::OffCommanded);
        igniter2.setState(PyroState::OffCommanded);
        }
        break;
    case EngineControllerState::FiringAutosequence:
        testPass = false;
        // I DO need this case to run every stateOperations cycle to check on sequence timing, or I need to move logic somewhere else
        //Fuel MV autosequence check
        if (currentAutosequenceTime >= fuelMVAutosequenceActuation) {pilotMVFuelValve.setState(ValveState::OpenCommanded);}
        else {pilotMVFuelValve.setState(ValveState::FireCommanded);}
        //Lox MV autosequence check
        if (currentAutosequenceTime >= loxMVAutosequenceActuation) {pilotMVLoxValve.setState(ValveState::OpenCommanded);}
        else {pilotMVLoxValve.setState(ValveState::FireCommanded);}
        //Engine Igniter1 autosequence check
        if (currentAutosequenceTime < igniter1Actuation) 
            {
            igniter1.setState(PyroState::FireCommanded);
            }        
        if (currentAutosequenceTime >= igniter1Actuation) 
            {
                if (igniter1.getState() == PyroState::FireCommanded)
                {
                    igniter1.setState(PyroState::OnCommanded);
                    igniter1timer = 0;
                }
                else if (igniter1.getState() == PyroState::OnCommanded && igniter1timer >= igniter1LiveOutTime)
                {
                    igniter1.setState(PyroState::OffCommanded);
                }
                else if (igniter1.getState() != PyroState::OffCommanded)
                {
                    {igniter1.setState(PyroState::OnCommanded);}
                }
            }
        else {igniter1.setState(PyroState::FireCommanded);}
        //Engine Igniter2 autosequence check
        if (currentAutosequenceTime < igniter2Actuation) 
            {
            igniter2.setState(PyroState::FireCommanded);
            }        
        if (currentAutosequenceTime >= igniter2Actuation) 
            {
                if (igniter2.getState() == PyroState::FireCommanded)
                {
                    igniter2.setState(PyroState::OnCommanded);
                    igniter2timer = 0;
                }
                else if (igniter2.getState() == PyroState::OnCommanded && igniter2timer >= igniter2LiveOutTime)
                {
                    igniter2.setState(PyroState::OffCommanded);
                }
                else if (igniter2.getState() != PyroState::OffCommanded)
                {
                    {igniter2.setState(PyroState::OnCommanded);}
                }
            }
        else {igniter2.setState(PyroState::FireCommanded);}
        // devices not conditional within autosequence below
        pneumaticVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Fast;
        break;
    case EngineControllerState::Shutdown:
        testPass = false;
        if (priorState != EngineControllerState::Shutdown)
        {
        sensorState = SensorState::Fast;
        pilotMVFuelValve.setState(ValveState::CloseCommanded);
        pilotMVLoxValve.setState(ValveState::CloseCommanded);
        pneumaticVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Fast;
        igniter1.setState(PyroState::OffCommanded);
        igniter2.setState(PyroState::OffCommanded);
        }
        break;
    case EngineControllerState::TestPassthrough:
        testPass = true;
        sensorState = SensorState::Slow;
        break;
    case EngineControllerState::OffNominalPassthrough:
        testPass = true;
        break;
    default:
        break;
    }
    pilotMVFuelValve.controllerStateOperations();
    pilotMVLoxValve.controllerStateOperations();
    pneumaticVent.controllerStateOperations();
    igniter1.controllerStateOperations();
    igniter2.controllerStateOperations();

}
