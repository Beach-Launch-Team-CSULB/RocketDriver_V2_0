#ifndef PyroClass_H
#define PyroClass_H

#include "PyroStates.h"
#include <Arduino.h>


class Pyro
{
    private:
        const uint32_t pyroID;
        const uint32_t pyroNodeID;
        const uint8_t firePin;
        const uint8_t armPin;
        elapsedMicros timer;
        PyroState state;
        PyroState priorState;
        const uint32_t liveOutTime;
        bool nodeIDCheck;                           // Whether this object should operate on this node

    public:
    
    // constructor, define the valve ID here, and the pin that controls the valve, setFireDelay is only parameter that can be left blank
        Pyro(uint32_t setPyroID, uint32_t setPyroNodeID, uint8_t setFirePin, uint8_t setArmPin, uint32_t setLiveOutTime, bool setNodeIDCheck = false); 
    // Alternate constructor with future full implementation, needs the clonedpyro features still
    //    Pyro(int setPyroID, int setPyroNodeID, int setFirePin, int setShuntPin, int setContPin, uint32_t setFireDelay = 0);

    // a start up method, to set pins from within setup()
        void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getPyroID(){return pyroID;}
        uint32_t getPyroNodeID(){return pyroNodeID;}
        uint32_t getFirePin(){return firePin;}
        uint32_t getArmPin(){return armPin;}
        //uint32_t getshuntPin(){return shuntPin;}
        //uint32_t getContPin(){return contCheckPin;}        
        uint32_t getLiveOutTime(){return liveOutTime;}
        PyroState getState(){return state;}
        uint32_t getTimer(){return timer;}
        bool getNodeIDCheck(){return nodeIDCheck;}

    // set functions, allows the setting of a variable
        //void setState(PyroState newState) {state = newState; timer = 0;} //every time a state is set, the timer should reset
        void setState(PyroState newState) 
            {
                if (newState != state)
                {
                    priorState = state;
                }
                state = newState;
            }    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}

    // functions with executables defined in ValveClasses.cpp
        void resetTimer();              // resets timer to zero, timer increments automatically in microseconds

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();



};

#endif