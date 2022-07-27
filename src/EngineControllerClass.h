#ifndef ENGINECONTROLLERCLASS_H
#define ENGINECONTROLLERCLASS_H

#include <Arduino.h>
#include "ControllerStates.h"
//#include "ValveStates.h"    // this is included also in ValveClass.h, hopefully it doesn't get me in trouble
#include "SensorStates.h"
//#include "PyroStates.h"
#include "ValveClass.h"
#include "PyroClass.h"

class EngineController
{
    private:
        const uint32_t controllerID;                          // Controller ID number 
        const uint8_t controllerNodeID;
        bool nodeIDCheck;                           // Whether this object should operate on this node
        bool testPass = false;
        EngineControllerState state;
        EngineControllerState priorState;
        SensorState sensorState;                    // Use one sensor state inside here to toggle all sensors on controller
        int64_t currentAutosequenceTime;
        int64_t fuelMVAutosequenceActuation;
        int64_t loxMVAutosequenceActuation;
        int64_t igniter1Actuation;
        int64_t igniter2Actuation;
        elapsedMicros timer;                        // timer for the valve, used for changing duty cycles, in MICROS
/*         ValveState pilotMVFuelValveState;
        ValveState pilotMVLoxValveState;
        ValveState pneumaticVentState;
        PyroState igniter1State;
        PyroState igniter2State; */

        Valve pilotMVFuelValve;
        Valve pilotMVLoxValve;
        Valve pneumaticVent;
        Pyro igniter1;
        Pyro igniter2;

        uint32_t igniter1LiveOutTime = 500000;
        uint32_t igniter2LiveOutTime = 500000;
        elapsedMicros igniter1timer = 0;
        elapsedMicros igniter2timer = 0;

    public:

    // constructor
        //EngineController(uint32_t setControllerID, uint8_t setControllerNodeID, int64_t fuelMVAutosequenceActuation = 0, int64_t loxMVAutosequenceActuation = 0, int64_t igniter1Actuation = 0, int64_t igniter2Actuation = 0, bool setNodeIDCheck = false);
    // constructor 2
        EngineController(uint32_t setControllerID, uint8_t setControllerNodeID, Valve* setPilotMVFuelValve, Valve* setPilotMVLoxValve, Valve* setPneumaticVent, Pyro* setIgniter1, Pyro* setIgniter2, int64_t fuelMVAutosequenceActuation = 0, int64_t loxMVAutosequenceActuation = 0, int64_t igniter1Actuation = 0, int64_t igniter2Actuation = 0, bool setNodeIDCheck = false);
    // a start up method, to set pins from within setup()
        void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getControllerID(){return controllerID;}
        uint8_t getControllerNodeID(){return controllerNodeID;}
        bool getNodeIDCheck(){return nodeIDCheck;}
        EngineControllerState getState(){return state;}
        EngineControllerState getPriorState(){return priorState;}
        ValveState getPilotMVFuelValveState(){return pilotMVFuelValve.getState();}
        ValveState getPilotMVLoxValveState(){return pilotMVLoxValve.getState();}
        PyroState getIgniter1State(){return igniter1.getState();}
        PyroState getIgniter2State(){return igniter2.getState();}
        ValveState getPneumaticVentState(){return pneumaticVent.getState();}

    // set functions, allows the setting of a variable
    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
    // controller state set function
        void setState(EngineControllerState newState)
            {
                if (newState != state)
                {
                    priorState = state;
                }
                state = newState;
            }
    //valve and pyro state set functions
        void setPilotMVFuelValveState(ValveState pilotMVFuelValveStateIn) {if (pilotMVFuelValveStateIn != ValveState::NullReturn){pilotMVFuelValve.setState(pilotMVFuelValveStateIn);}}
        void setPilotMVLoxValveState(ValveState pilotMVLoxValveStateIn) {if (pilotMVLoxValveStateIn != ValveState::NullReturn){pilotMVLoxValve.setState(pilotMVLoxValveStateIn);}}
        void setIgniter1State(PyroState igniter1StateIn) {if (igniter1StateIn != PyroState::NullReturn){igniter1.setState(igniter1StateIn);}}
        void setIgniter2State(PyroState igniter2StateIn) {if (igniter2StateIn != PyroState::NullReturn){igniter2.setState(igniter2StateIn);}}


        void testSetPilotMVFuelValveState(ValveState pilotMVFuelValveStateIn) {if(testPass) {pilotMVFuelValve.setState(pilotMVFuelValveStateIn);}}
        void testSetPilotMVLoxValveState(ValveState pilotMVLoxValveStateIn) {if(testPass) {pilotMVLoxValve.setState(pilotMVLoxValveStateIn);}}
        void testSetIgniter1State(PyroState igniter1StateIn) {if(testPass) {igniter1.setState(igniter1StateIn);}}
        void testSetIgniter2State(PyroState igniter2StateIn) {if(testPass) {igniter2.setState(igniter2StateIn);}}


    // autosequence get function
        void setCurrentAutosequenceTime(int64_t countdownIn) {currentAutosequenceTime = countdownIn;}
    
    // functions with executables defined in ValveClasses.cpp
        void resetTimer();              // resets timer to zero, timer increments automatically in microseconds

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();

};


#endif