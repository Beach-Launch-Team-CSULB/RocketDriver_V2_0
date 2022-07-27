#ifndef FLUIDSYSTEMSIMULATION_H
#define FLUIDSYSTEMSIMULATION_H

#include <Arduino.h>
#include "ValveStates.h"


int unitConversionCosnt = 6895;
float AirGasConstant = 287; //J/kg-k
float ATPtemp = 288.15; //K
float Gamma = 1.4;
float Cd = 0.973;



class tankObject
{
    public:
    float TankVolume = 0; //m^3
    float OptimalTankPress;
    float TankMass = 0;
    float CurrPressure = 0;
    float TankPressure = 0;
    float gasDensity = 0;
    ValveState inletValveState = ValveState::Closed;
    ValveState outletValveState = ValveState::Closed;
    ValveState ventValveState = ValveState::Closed;

    float CurrTankPress(float TankPropMass);
    void pressureUpdateFunction(float TimeDelta);
};


class FluidSystemSimulation
{
    private:
        tankObject HiPressTank;
        tankObject FuelTank;
        tankObject LoxTank;
        
        float TimeDelta;


    public:

    void fluidSystemUpdate();

};


#endif