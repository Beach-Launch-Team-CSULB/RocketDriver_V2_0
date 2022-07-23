#ifndef ENGINECONTROLLERCLASS_H
#define ENGINECONTROLLERCLASS_H

#include <Arduino.h>
#include "ControllerStates.h"
#include "ValveStates.h"    // this is included also in ValveClass.h, hopefully it doesn't get me in trouble
#include "SensorStates.h"
#include "PyroStates.h"

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
        ValveState pilotMVFuelValveState;
        ValveState pilotMVLoxValveState;
        ValveState pneumaticVentState;
        PyroState igniter1State;
        PyroState igniter2State;
        uint32_t igniter1LiveOutTime = 500000;
        uint32_t igniter2LiveOutTime = 500000;
        elapsedMicros igniter1timer = 0;
        elapsedMicros igniter2timer = 0;

    public:

    // constructor
        EngineController(uint32_t setControllerID, uint8_t setControllerNodeID, int64_t fuelMVAutosequenceActuation = 0, int64_t loxMVAutosequenceActuation = 0, int64_t igniter1Actuation = 0, int64_t igniter2Actuation = 0, bool setNodeIDCheck = false);
    // a start up method, to set pins from within setup()
        void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getControllerID(){return controllerID;}
        uint8_t getControllerNodeID(){return controllerNodeID;}
        bool getNodeIDCheck(){return nodeIDCheck;}
        EngineControllerState getState(){return state;}
        ValveState getPilotMVFuelValveState(){return pilotMVFuelValveState;}
        ValveState getPilotMVLoxValveState(){return pilotMVLoxValveState;}
        PyroState getIgniter1State(){return igniter1State;}
        PyroState getIgniter2State(){return igniter2State;}
        ValveState getPneumaticVentState(){return pneumaticVentState;}

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