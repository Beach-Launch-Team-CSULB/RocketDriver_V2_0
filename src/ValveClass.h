#ifndef VALVECLASS_H
#define VALVECLASS_H

#include <Arduino.h>
#include "ValveStates.h"

// This class defines the Valve Object that will be used to represent and actuate the valves
// Run begin to set the pins

enum ValveType
{
    NormalClosed,
    NormalOpen,
};

class Valve
{

    private:
        const uint32_t valveID = 99;                          // Valve ID number 
        const uint8_t valveNodeID = 99;                      // NodeID the valve is controlled by
        ValveType valveType;                  // sets the valve type, either normal closed or normal open
        const uint8_t pinPWM = 99;                              // Valve PWM pin for actuation
        const uint8_t pinDigital = 99;                          // Valve Digital Out pin for actuation
        uint32_t fullDutyTime = 100;                // Time PWM needs to be at full duty for actuation, in MICROS
        ValveState state;
        ValveState priorState;                           // Tracks the valve state
        elapsedMicros timer;                        // timer for the valve, used for changing duty cycles, in MICROS
        uint16_t fullDuty{256};                // full duty cycle for servo initial actuation
        uint8_t holdDuty{};                   // partial duty cycle to hold valve in actuated state
        bool nodeIDCheck;                           // Whether this object should operate on this node
        bool abortHaltDeviceBool;                    // Whether this valve is set by the abort halt flag override
        ValveState abortedState;
        uint16_t controlSensor1Value;               // For use in control schemes, really a template placement pending needed number and type of samples
        bool controllerUpdate = false;              // flag for when valve stateOps does a state change to update in the controllers

    public:
    
    // constructor, define the valve ID here, and the pin that controls the valve, setFireDelay is only parameter that can be left blank
        Valve(uint32_t setValveID, uint8_t setValveNodeID, ValveType setValveType, uint8_t setPinPWM, uint8_t setPinDigital, uint32_t setFullDutyTime,  
        bool setAbortHaltDeviceBool = false, ValveState setAbortedState = ValveState::CloseCommanded, uint8_t setHoldDuty = 64, bool setNodeIDCheck = false);
    // Default constructor with no args    
        Valve(ValveType setValveType);
    // a start up method, to set pins from within setup()
        void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getValveID(){return valveID;}
        uint8_t getValveNodeID(){return valveNodeID;}
        ValveType getValveType(){return valveType;}
        uint8_t getPinPWM(){return pinPWM;}
        uint8_t getPinDigital(){return pinDigital;}
        uint32_t getFullDutyTime(){return fullDutyTime;}
        uint8_t getHoldDuty(){return holdDuty;}
        ValveState getState(){return state;}
        ValveState getSyncState();
        ValveState getPriorState(){return priorState;}
        ValveState getAbortedState(){return abortedState;}
        uint32_t getTimer(){return timer;}
        bool getNodeIDCheck(){return nodeIDCheck;}
        bool getAbortHaltDeviceBool(){return abortHaltDeviceBool;}

    // set functions, allows the setting of a variable
        void setState(ValveState newState) 
            {
                if (newState == ValveState::OpenCommanded)
                {
                    if (priorState == ValveState::OpenProcess || priorState == ValveState::Open)
                    {
                        //Don't update the state, it's already doing opening operations
                    }
                    else
                    {
                    if (newState != state)
                    {
                        priorState = state;
                    }
                    state = newState;
                    }
                }
                else if (newState == ValveState::CloseCommanded)
                {
                    if (priorState == ValveState::CloseProcess || priorState == ValveState::Closed)
                    {
                        //Don't update the state, it's already doing closing operations
                    }
                    else
                    {
                    if (newState != state)
                    {
                        priorState = state;
                    }
                    state = newState;
                    }
                }
                else
                {
                if (newState != state)
                {
                    priorState = state;
                }
                state = newState;
                }
            }
            
        //every time a state is set, the timer should reset
        //Is the above still true?

    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
    //set functions 
        void setValveType(uint8_t typeIn){if (typeIn == 0 || typeIn == 1) {valveType = static_cast<ValveType>(typeIn);}}
        void setFullDutyTime(uint32_t fullDutyTimeIn){if (fullDutyTimeIn <= 10000) {fullDutyTime = fullDutyTimeIn;}}

    // functions with executables defined in ValveClasses.cpp
        void resetTimer();              // resets timer to zero, timer increments automatically in microseconds

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();
        void controllerStateOperations();

    // Sensor pull in function for control
        //void controlSensorFetch(uint16_t updateControlSensor1Value){controlSensor1Value = updateControlSensor1Value;}
};

#endif