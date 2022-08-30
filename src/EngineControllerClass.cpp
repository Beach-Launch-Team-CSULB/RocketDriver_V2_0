#include "EngineControllerClass.h"
#include <Arduino.h>

EngineController::EngineController(uint32_t setControllerID, uint8_t setControllerNodeID, float setCurrentPcTarget_Default, Valve* setPilotMVFuelValve, Valve* setPilotMVLoxValve, Valve* setPneumaticVent, Pyro* setIgniter1, Pyro* setIgniter2, int64_t setFuelMVAutosequenceActuation, int64_t setLoxMVAutosequenceActuation, int64_t setIgniter1Actuation, int64_t setIgniter2Actuation, bool setNodeIDCheck) 
                                        : controllerID{setControllerID}, controllerNodeID{setControllerNodeID}, currentPcTarget_Default{setCurrentPcTarget_Default}, pilotMVFuelValve{*setPilotMVFuelValve}, pilotMVLoxValve{*setPilotMVLoxValve}, pneumaticVent{*setPneumaticVent}, igniter1{*setIgniter1}, igniter2{*setIgniter2}, fuelMVAutosequenceActuation{setFuelMVAutosequenceActuation}, loxMVAutosequenceActuation{setLoxMVAutosequenceActuation}, igniter1Actuation{setIgniter1Actuation}, igniter2Actuation{setIgniter2Actuation}, nodeIDCheck{setNodeIDCheck}
{
    fuelMVAutosequenceActuation = fuelMVAutosequenceActuation_Default;
    loxMVAutosequenceActuation = loxMVAutosequenceActuation_Default;
    igniter1Actuation_Default = igniter1Actuation;
    igniter2Actuation = igniter2Actuation_Default;
    currentPcTarget = currentPcTarget_Default;

    //move below into begin???
    throttleProgram.reserve(25); // allocates memory for 25 throttle points
    throttlePoint initialThrottlePoint = {0, currentPcTarget};
    throttleProgram.push_back(initialThrottlePoint);
    //temp throttle points
    throttlePoint testSecondThrottlePoint = {5000000, 290};
    //throttlePoint testSecondThrottlePoint = {2000000, 290};
    throttleProgram.push_back(testSecondThrottlePoint);
    //throttlePoint testThirdThrottlePoint = {4000000, 330};
    throttlePoint testThirdThrottlePoint = {10000000, 330};
    throttleProgram.push_back(testThirdThrottlePoint);
    //throttle program iterator
    throttleProgramPos = throttleProgram.begin(); //starts the iterator at first position of array
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

void EngineController::resetAll()
{
    fuelMVAutosequenceActuation = fuelMVAutosequenceActuation_Default;
    loxMVAutosequenceActuation = loxMVAutosequenceActuation_Default;
    igniter1Actuation_Default = igniter1Actuation;
    igniter2Actuation = igniter2Actuation_Default;

    currentPcTarget = currentPcTarget_Default;
}

bool EngineController::throttlePointCheck(throttlePoint &pt, vector<throttlePoint> &throttleProgram)
{
    //function iterates through vector to find any points with same time value
    //if time value existed it overwrites with new target Pc, deletes any duplicates
    //return is false if time value was not present, means need to insert throrrle point at correct location still
    
    bool isPointAlreadyPresent = false;
    //throttlePoint currentPosition;
    for (auto i = throttleProgram.begin(); i != throttleProgram.end(); ++i)
    {
        //currentPosition = throttleProgram.at(i).autoSequenceTimeValue;
        if (pt.autoSequenceTimeValue == (i->autoSequenceTimeValue))
        {
            if (!isPointAlreadyPresent)
            {
            //finds first instance the element exists, overwrites the Pc for the given time point
            i->targetPcValue = pt.targetPcValue;
            isPointAlreadyPresent = true;
            //Serial.println(" found 1 entry and updated: ");
            }
            else
            {
                //erases any duplicated time points
                //Serial.println(" found and killed duplicate: ");
                throttleProgram.erase(i);
            }
        }
    }
    return isPointAlreadyPresent;
}

//void EngineController::setThrottleProgramPoint(uint16_t autoSequenceTimeMillisIn, uint16_t currentPcTargetIn)
void EngineController::setThrottleProgramPoint(uint16_t autoSequenceTimeMillisIn, uint16_t currentPcTargetIn)
{
    throttlePoint ptIn;
    //only can change throttle vector while controller isn't in active state
    //function should work correctly in any state, but normally do not want to be changing the throttleprogram once running
    if(state == EngineControllerState::Passive) 
    {    
        if(currentPcTargetIn >= 200 && currentPcTargetIn <= 600)
        {
            //vector version
            if (autoSequenceTimeMillisIn >= 0 && autoSequenceTimeMillisIn <= 60000)
            {
                ptIn.autoSequenceTimeValue = static_cast<int64_t>(autoSequenceTimeMillisIn*1000);
                ptIn.targetPcValue = float(currentPcTargetIn);
            
            if (!throttlePointCheck(ptIn,throttleProgram) && (throttleProgram.size() != throttleProgram.capacity()))
            {
                auto itPos = throttleProgram.begin();
                for (auto i = throttleProgram.begin(); i != throttleProgram.end(); ++i)
                {
                    if (ptIn.autoSequenceTimeValue > (i->autoSequenceTimeValue))
                    {
                        //Serial.print("ptIn.autoSequenceTimeValue : ");
                        Serial.print(ptIn.autoSequenceTimeValue);
                        //Serial.print("ptIn.targetPcValue : ");
                        Serial.print(ptIn.targetPcValue);
                        //Serial.println(" DO i MAKE IT TO BREAK : ");
                        ++itPos;
                        //break;
                    }
                }
                        throttleProgram.insert((itPos),ptIn);
            }
            }

        }
        throttleProgramPos = throttleProgram.begin(); //sets/resets throttle program iterator
    }
}

void EngineController::throttleProgramReset()
{
    if(state == EngineControllerState::Passive)
    { 
        // no input args == reset whole program
        throttleProgram.clear();
        //clear wipes everything, invalidates all iterators
        //add back default T=0 point
        throttlePoint initialThrottlePoint = {0, currentPcTarget_Default};
        throttleProgram.push_back(initialThrottlePoint);
        // restarts the iterator
        throttleProgramPos = throttleProgram.begin(); //starts the iterator at first position of array
    }
}

void EngineController::throttleProgramReset(uint16_t autoSequenceTimeMillisIn)
{
    if(state == EngineControllerState::Passive)
    {
    // autosequence time input arg == remove a point at that time if it exists
    for (auto i = throttleProgram.begin(); i != throttleProgram.end(); ++i)
    {
        //checks for any matchs and erases them
        if (static_cast<int64_t>(autoSequenceTimeMillisIn*1000) == (i->autoSequenceTimeValue))
        {
            throttleProgram.erase(i);
        }
    }
        //checks to see if vector is now empty, and if so puts back default T=0 value to not have invalid throttle sequence
        if (throttleProgram.empty())
        {
            //add back default T=0 point
            throttlePoint initialThrottlePoint = {0, currentPcTarget_Default};
            throttleProgram.push_back(initialThrottlePoint);
            // restarts the iterator
            throttleProgramPos = throttleProgram.begin(); //starts the iterator at first position of array
        }

    }
}

void EngineController::autoSequenceTargetPcUpdate(bool runBool)
{
    //attempting to make it so progressing through throttle program updates the iterator so each function call should start on the next value
    if (runBool)
    {
        for (; throttleProgramPos != throttleProgram.end();)
        //for (auto throttleProgramPos = throttleProgram.begin(); != throttleProgram.end();) //old version that doesn't pick up where we left off
        {
            if (currentAutosequenceTime >= (throttleProgramPos->autoSequenceTimeValue))
            {
                currentPcTarget = (throttleProgramPos)->targetPcValue;
                ++throttleProgramPos;
                if (throttleProgramPos == throttleProgram.end())    //DIDNT work it bricked and didn't update iterator ever
                {
                    //if we reach the end, go back 1 intdex
                    --throttleProgramPos;
                }
            }
            break;
        }
    }
    else    //if not set to run, grabs the first element in the throttle array to get tank press target
    {
        currentPcTarget = throttleProgram.begin()->targetPcValue;
    }
}
void EngineController::stateOperations()
{
    switch (state)
    {
    case EngineControllerState::Passive:
        testPass = false;
        //don't do shit
        throttleProgramPos = throttleProgram.begin(); //sets/resets throttle program iterator
        autoSequenceTargetPcUpdate(false);
        //if (priorState != EngineControllerState::Passive)
        //{
        pilotMVFuelValve.setState(ValveState::CloseCommanded);
        pilotMVLoxValve.setState(ValveState::CloseCommanded);
        pneumaticVent.setState(ValveState::CloseCommanded);
        sensorState = SensorState::Slow;
        //}
        MVFuelFired = false;
        MVLoxFired = false;
        igniter1.resetPyro();
        igniter2.resetPyro();
        break;
    case EngineControllerState::Armed:
        testPass = false;
        // Arming turns sensor read rates up to operational levels before opening valves
        autoSequenceTargetPcUpdate(true);
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
        autoSequenceTargetPcUpdate(true);
        // I DO need this case to run every stateOperations cycle to check on sequence timing, or I need to move logic somewhere else
        if (!MVFuelFired)
        {
            //Fuel MV autosequence check
            if (currentAutosequenceTime >= fuelMVAutosequenceActuation) {pilotMVFuelValve.setState(ValveState::OpenCommanded); MVFuelFired = true;}
            else {pilotMVFuelValve.setState(ValveState::FireCommanded);}
        }
        if (!MVLoxFired)
        {
            //Lox MV autosequence check
            if (currentAutosequenceTime >= loxMVAutosequenceActuation) {pilotMVLoxValve.setState(ValveState::OpenCommanded); MVLoxFired = true;}
            else {pilotMVLoxValve.setState(ValveState::FireCommanded);}
        }
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
                    //igniter1timer = 0;
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
                    //igniter2timer = 0;
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
