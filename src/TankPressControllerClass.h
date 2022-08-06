#ifndef TANKPRESSCONTROLLERCLASS_H
#define TANKPRESSCONTROLLERCLASS_H

#include <Arduino.h>
#include "ControllerStates.h"
#include "SensorStates.h"
#include "ValveClass.h"

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
        ValveState pressLineVentStateBang1;
        ValveState pressLineVentStateBang2;
        Valve &primaryPressValve;
        Valve &pressLineVent;
        Valve &tankVent;
        //Valve MainValve{};
        bool abortFlag = false;         //controller can trigger an abort by flipping this flag true
        bool ventFailsafeFlag = false;  //for making vent failsafe require successive controller loops to open vents
        float ventFailsafePressure;
        float ventFailsafePressure_Default;
        float targetPcValue;
        float targetPcValue_Default;
        float tankToChamberDp;
        float tankToChamberDp_Default;

        //float targetValue_Default;
        float K_p_Default = 1;
        float K_i_Default = 0; //initial Ki, KEEP 0 for tank press
        float K_i_run_Default = 0; //bang run Ki
        float K_d_Default = 0;
        float controllerThreshold_Default = 1;

        
        float targetValue;
        float K_p = 1;
        float K_i = 0; //initial Ki, KEEP 0 for tank press
        float K_i_run = 0; //bang run Ki
        float K_d = 0;
        float controllerThreshold = 1;
        float bangPIDoutput;

        uint32_t valveMinimumEnergizeTime_Default = 75;      // in ms
        uint32_t valveMinimumDeenergizeTime_Default = 50;    // in ms
        uint32_t valveMinimumEnergizeTime = 75;      // in ms
        uint32_t valveMinimumDeenergizeTime = 50;    // in ms
        float controllerTimeStep = 0.01; //default to 100Hz assumption for controller refresh
        float sensorIntervalTimeStep = 0.01;

        bool trustBangSensor1 = false;
        bool trustBangSensor2 = false;
        bool trustBangSensor3 = true;
        float bangSensor1EMA = 0;   //primary PT
        float bangSensor2EMA = 0;   //secondary PT
        float bangSensor3EMA = 0;   //simulated PT
        float bangSensorWeightedEMA = 0;   //weighted ave/trusted value - currently crude use
        float bangSensor1Integral = 0;   //primary PT
        float bangSensor2Integral = 0;   //secondary PT
        float bangSensor3Integral = 0;   //simulated PT
        float bangSensor1Derivative = 0;   //primary PT
        float bangSensor2Derivative = 0;   //secondary PT
        float bangSensor3Derivative = 0;   //simulated PT
        
        float proportionalValue = 0;
        float integralValue = 0;
        float derivativeValue = 0;

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

    public:

    // constructor - hipress
        TankPressController(uint32_t controllerID, uint8_t setControllerNodeID, Valve* primaryPressValve, Valve* pressLineVent, Valve* tankVent, float setVentFailsafePressure_Default, bool setNodeIDCheck = false);
    // constructor - tank bangers
        TankPressController(uint32_t controllerID, uint8_t setControllerNodeID, Valve* primaryPressValve, Valve* pressLineVent, Valve* tankVent, float setTargetPcValue_Default, float setTankToChamberDp_Default, float setVentFailsafePressure_Default, float set_K_p_Default, float set_K_i_Default, float set_K_d_Default, float setControllerThreshold_Default, bool isSystemBang = true, bool setNodeIDCheck = false);
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
        SensorState getControllerSensorState(){return sensorState;}
        ValveState getPrimaryPressValveState(){return primaryPressValve.getState();}
        ValveState getPressLineVentState(){return pressLineVent.getState();}
        ValveState getTankVentState(){return tankVent.getState();}
        bool getAbortFlag(){return abortFlag;}

        bool getResetIntegralCalcBool()
            {
                bool tempBoolContainer;
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
        void setPIDSensorInput1(float proportionalValue, float integralValue, float derivativeValue);
        void setPIDSensorInput2(float proportionalValue, float integralValue, float derivativeValue);
        void setPIDSensorInput3(float proportionalValue, float integralValue, float derivativeValue);

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

        void setPrimaryPressValveState(ValveState primaryPressValveStateIn) {if (primaryPressValveStateIn != ValveState::NullReturn) {primaryPressValve.setState(primaryPressValveStateIn);}}
        void setPressLineVentState(ValveState pressLineVentStateIn) {if (pressLineVentStateIn != ValveState::NullReturn) {pressLineVent.setState(pressLineVentStateIn);}}
        void setTankVentState(ValveState tankVentStateIn) {if (tankVentStateIn != ValveState::NullReturn) {tankVent.setState(tankVentStateIn);}}
        //test state set functions
        void testSetPrimaryPressValveState(ValveState primaryPressValveStateIn) {if(testPass) {primaryPressValve.setState(primaryPressValveStateIn);}}
        void testSetPressLineVentState(ValveState pressLineVentStateIn) {if(testPass) {pressLineVent.setState(pressLineVentStateIn);}}
        void testSetTankVentState(ValveState tankVentStateIn) {if(testPass) {tankVent.setState(tankVentStateIn);}}
    //setting functions - have all inputs bounded to catch nonsense CAN config msg inputs
        void setVentFailsafePressure(float ventFailsafePressureIn){if (ventFailsafePressureIn <= 10000 && ventFailsafePressureIn >= 0) {ventFailsafePressure = ventFailsafePressureIn;}}
        
        void setK_p(float K_pin){if (K_pin <= 1000 && K_pin >= -1000) {K_p = K_pin;}}
        void setK_i(float K_iin){if (K_iin <= 1000 && K_iin >= -1000) {K_i = K_iin;}}
        void setK_i(){K_i = K_i_run;}   //empty input args means reset K_i to K_i_run
        void setK_d(float K_din){if (K_din <= 1000 && K_din >= -1000) {K_d = K_din;}}
        void setControllerThreshold(float controllerThresholdIn){if (controllerThresholdIn <= 100 && controllerThresholdIn >= 0) {controllerThreshold = controllerThresholdIn;}}
        void setValveMinimumEnergizeTime(uint32_t valveMinimumEnergizeTimeIn){if(valveMinimumEnergizeTimeIn >= 0 && valveMinimumEnergizeTimeIn <= 10000){valveMinimumEnergizeTime = valveMinimumEnergizeTimeIn;}}
        void setValveMinimumDeenergizeTime(uint32_t valveMinimumDeenergizeTimeIn){if(valveMinimumDeenergizeTimeIn >= 0 && valveMinimumDeenergizeTimeIn <= 10000){valveMinimumDeenergizeTime = valveMinimumDeenergizeTimeIn;}}

        void setPcTarget(float PcTargetIn);
    
    // reset all configurable settings to defaults
        void resetAll();
    // functions with executables defined in ValveClasses.cpp
        void resetTimer();              // resets timer to zero, timer increments automatically in microseconds
    // autosequence get function
        void setCurrentAutosequenceTime(int64_t countdownIn) {currentAutosequenceTime = countdownIn;}


    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();

        void PIDinputSetting(); //logic for which input source to use for PID values
        float PIDmath();
    
    // controller set point function
        void setControllerTargetValue(float controllerSetPointIn){targetValue = controllerSetPointIn;}

        void ventPressureCheck();
};


#endif