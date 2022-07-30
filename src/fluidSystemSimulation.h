#ifndef FLUIDSYSTEMSIMULATION_H
#define FLUIDSYSTEMSIMULATION_H

#include <Arduino.h>
#include "ValveStates.h"

ValveState valveStateFlowSimSimplify(ValveState inputValveState);

class PressurantTank
{

    public:
    double CurrPressure = 6000 * 6895;
    //double TankVolume = .05; // m^3 K bottle
    double TankVolume = .001; // m^3 paintball COPV
    double PressurantMass=85;
    double CdA = 0.0000000645;
    double massFlow = 0;
    
    double ChokedMassFlow(double TimeDelta);
    
    void pressureUpdateFunction(double TimeDelta, double pressMFtank1, double pressMFtank2);
    
    PressurantTank();

};


class tankObject
{
    public:
    double UllageVolume = 0.005; //m^3
    double CurrPressure = 15; //BSing an ambient start pressure?
    double UllageMass;
    double OutflowCdA;
    double pressMassFlow = 0;
    double massFlow = 0;
    ValveState inletValveState = ValveState::Closed;
    ValveState outletValveState = ValveState::Closed;
    ValveState ventValveState = ValveState::Closed;

    void SetValveStates(ValveState InState, ValveState OutState, ValveState VentState);

    void IncompressibleMassFlow(double TimeDelta);

    void pressureUpdateFunction(double TimeDelta, PressurantTank PressTank);

    double getCurrPressure(){return CurrPressure;}//if currpressure stays public don't need get function

    tankObject(double setOutflowCdA);
};



class FluidSystemSimulation
{
    public:
        PressurantTank HiPressTank;
        tankObject FuelTank;
        tankObject LoxTank;
        elapsedMicros simTimeEllapsed;
        double TimeDelta;
        FluidSystemSimulation(double setTimeDelta, PressurantTank setHiPressTank, tankObject setFuelTank, tankObject setLoxTank);
    
    // Top Level Fluid Simulation function, run each time step
    void fluidSystemUpdate();
    // Fake analog sensor read function, used for feeding the propulsion controllers simulated data
    float analogRead(uint8_t fakeADCpin);
};



#endif