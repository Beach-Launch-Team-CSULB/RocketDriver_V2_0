#ifndef TANKPRESSCONTROLLERCLASS_H
#define TANKPRESSCONTROLLERCLASS_H

#include <Arduino.h>
#include "ControllerStates.h"
//#include <array>
#include "ValveStates.h"    // this is included also in ValveClass.h, hopefully it doesn't get me in trouble
#include "SensorStates.h"
#include "ControllerMathFunctions.h"

class TankPressController
{
    private:
        const uint32_t controllerID;                          // Controller ID number 
        const uint8_t controllerNodeID;
        bool nodeIDCheck;                           // Whether this object should operate on this node
        bool isSystemBang;
        bool testPass = false;
        TankPressControllerState state;
        TankPressControllerState priorState;
        int64_t currentAutosequenceTime;
        SensorState sensorState;                    // Use one sensor state inside here to toggle all sensors on controller
        elapsedMicros timer;                        // timer for the valve, used for changing duty cycles, in MICROS
        elapsedMicros bangtimer;                        // timer for the valve, used for changing duty cycles, in MICROS
        ValveState primaryPressValveState;
        ValveState pressLineVentState;
        ValveState tankVentState;
        ValveState MainValveState;      //not for controlling, but for use as an input
        ValveState pressLineVentStateBang1;
        ValveState pressLineVentStateBang2;
        float ventFailsafePressure;

        float targetValue;
        float K_p = 1;
        float K_i = 0; //initial Ki, KEEP 0 for tank press
        float K_i_run = 0; //bang run Ki
        float K_d = 0;
        float controllerThreshold = 1;
        float bangPIDoutput;

        uint32_t valveMinimumEnergizeTime = 75;      // in ms
        uint32_t valveMinimumDeenergizeTime = 50;    // in ms
        float controllerTimeStep = 0.01; //default to 100Hz assumption for controller refresh
        float sensorIntervalTimeStep = 0.01;

        float bangSensor1EMA = 0;   //primary PT
        float bangSensor2EMA = 0;   //secondary PT
        float bangSensor3EMA = 0;   //simulated PT
        
        //do i need to create a float array for sensors here or can I pass a reference/similar?
        float funcOutput = 0;
        float p_rollingAve = 0;
        float P_p = 0;
        float P_i = 0;
        float P_d = 0;
        float e_p = 0;
        float e_i = 0;
        float e_d = 0;
        int arrayMostRecentPositionPID = 0;
        bool PIDmathPrintFlag = false;
        float timeStepPIDMath = 0;

        bool resetIntegralCalcBool = false;
        bool tempBoolContainer = false;

    public:

    // constructor - hipress
        TankPressController(uint32_t controllerID, uint8_t setControllerNodeID, float setTargetValue, float setVentFailsafePressure, bool setNodeIDCheck = false);
    // constructor - tank bangers
        TankPressController(uint32_t controllerID, uint8_t setControllerNodeID, float setTargetValue, float setVentFailsafePressure, float set_K_p, float set_K_i, float set_K_d, float setControllerThreshold, bool isSystemBang = true, bool setNodeIDCheck = false);
    // a start up method, to set pins from within setup()
        void begin();

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getControllerID(){return controllerID;}
        uint8_t getControllerNodeID(){return controllerNodeID;}
        bool getNodeIDCheck(){return nodeIDCheck;}
        bool getIsBang(){return isSystemBang;}
        float getTargetValue(){return targetValue;}
        TankPressControllerState getState(){return state;}
        TankPressControllerState getPriorState(){return priorState;}
        ValveState getPrimaryPressValveState(){return primaryPressValveState;}
        ValveState getPressLineVentState(){return pressLineVentState;}
        ValveState getTankVentState(){return tankVentState;}
        bool getResetIntegralCalcBool()
            {
                tempBoolContainer = resetIntegralCalcBool;
                resetIntegralCalcBool = false;              //default resets it to false
                return tempBoolContainer;
            }
        
        float getPfunc(){return e_p;}
        float getIfunc(){return e_i;}
        float getDfunc(){return e_d;}
        float getKp(){return K_p;}
        float getKi(){return K_i;}
        float getKd(){return K_d;}
        float getPIDoutput(){return bangPIDoutput;}

    // set functions, allows the setting of a variable
        void setPIDSensorInputs(float proportionalValue, float integralValue, float derivativeValue);
        void resetIntegralCalc(bool resetIntIn){resetIntegralCalcBool = resetIntIn;}
    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
    // controller state set function
        void setState(TankPressControllerState newState)
            {
                if (newState != state)
                {
                    priorState = state;
                }
                state = newState;
            }        
    // vent line setting - for bang bang with two tank controllers sharing vent line control
        void setPressVentLineStateBang1(ValveState ventLineSetIn) {pressLineVentStateBang1 = ventLineSetIn;}
    // vent line setting - for bang bang with two tank controllers sharing vent line control
        void setPressVentLineStateBang2(ValveState ventLineSetIn) {pressLineVentStateBang2 = ventLineSetIn;}

        void setPrimaryPressValveState(ValveState primaryPressValveStateIn) {primaryPressValveState = primaryPressValveStateIn;}
        void setPressLineVentState(ValveState pressLineVentStateIn) {pressLineVentState = pressLineVentStateIn;}
        void setTankVentState(ValveState tankVentStateIn) {tankVentState = tankVentStateIn;}
        //test state set functions
        void testSetPrimaryPressValveState(ValveState primaryPressValveStateIn) {if(testPass) {primaryPressValveState = primaryPressValveStateIn;}}
        void testSetPressLineVentState(ValveState pressLineVentStateIn) {if(testPass) {pressLineVentState = pressLineVentStateIn;}}
        void testSetTankVentState(ValveState tankVentStateIn) {if(testPass) {tankVentState = tankVentStateIn;}}
    //setting PID parameters
        void setK_p(float K_pin){K_p = K_pin;}
        void setK_i(float K_iin){K_i = K_iin;}
        void setK_i(){K_i = K_i_run;}   //empty input args means reset K_i to K_i_run
        void setK_d(float K_din){K_d = K_din;}

    // functions with executables defined in ValveClasses.cpp
        void resetTimer();              // resets timer to zero, timer increments automatically in microseconds
    // autosequence get function
        void setCurrentAutosequenceTime(int64_t countdownIn) {currentAutosequenceTime = countdownIn;}


    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();

        //float PIDmath(float inputArrayPID[], float controllerSetPoint, float timeStepPIDMath, float integrationSteps, float errorThreshold, float K_p, float K_i, float K_d);
        float PIDmath();
    
    // controller set point function
        void setControllerTargetValue(float controllerSetPointIn){targetValue = controllerSetPointIn;}

        void ventPressureCheck();
};


#endif