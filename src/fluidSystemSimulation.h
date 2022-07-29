#ifndef FLUIDSYSTEMSIMULATION_H
#define FLUIDSYSTEMSIMULATION_H

#include <Arduino.h>
#include "ValveStates.h"

ValveState valveStateFlowSimSimplify(ValveState inputValveState);

class PressurantTank
{

    public:
    float CurrPressure = 6000;
    float TankVolume = 5; // m^3
    float PressurantMass = 0;
    float CdA = 0.01;

    float ChokedMassFlow(float TimeDelta);

    PressurantTank();

};


class tankObject
{
    public:
    float UllageVolume = 0.005; //m^3
    float OptimalTankPress = 150;
    float CurrPressure = 15; //BSing an ambient start pressure?
    float UllageMass = 0;
    float OutflowCdA;

    ValveState inletValveState = ValveState::Closed;
    ValveState outletValveState = ValveState::Closed;
    ValveState ventValveState = ValveState::Closed;

    void SetValveStates(ValveState InState, ValveState OutState, ValveState VentState)
    {
    ValveState inletValveState = valveStateFlowSimSimplify(InState);
    ValveState outletValveState = valveStateFlowSimSimplify(OutState);
    ValveState ventValveState = valveStateFlowSimSimplify(VentState);
    };

    void IncompressibleMassFlow(float TimeDelta);

    void pressureUpdateFunction(float TimeDelta, PressurantTank PressTank);

    float getCurrPressure(){return CurrPressure;}//if currpressure stays public don't need get function

    tankObject(float setOutflowCdA);
};



class FluidSystemSimulation
{
    public:
        PressurantTank HiPressTank;
        tankObject FuelTank;
        tankObject LoxTank;
        float TimeDelta;
        FluidSystemSimulation(float setTimeDelta, PressurantTank setHiPressTank, tankObject setFuelTank, tankObject setLoxTank);
    void fluidSystemUpdate();
    float analogRead(uint8_t fakeADCpin);
};



#endif