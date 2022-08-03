#ifndef FLUIDSYSTEMSIMULATION_H
#define FLUIDSYSTEMSIMULATION_H

#include <Arduino.h>
#include "ValveStates.h"

ValveState valveStateFlowSimSimplify(ValveState inputValveState);

class PressurantTank
{

    public:
    double startPressure = 6000; // in PSI
    double CurrPressure;        // in Pa
    double KbottleTankVolume = .05; // m^3 K bottle
    double COPVTankVolume = .001; // m^3 paintball COPV
    double TankVolume = .001; // m^3 
    double PressurantMass;
    double CdA = 0.000000203; //0000000645
    double massFlow = 0;
    
    double ChokedMassFlow(double TimeDelta);
    
    void pressureUpdateFunction(double TimeDelta, double pressMFtank1, double pressMFtank2);
    
    void resetTankObject(); // for restarting sim

    PressurantTank();

};

enum class propFluid
{
    Lox,
    IPA,
    denatAlch,
    Kero,
    Water,
};

class tankObject
{
    public:
    propFluid tankFluid;
    float PropDensity = 999; //water default value
    double UllageVolumeStart = 0.001; //m^3 0.0005
    double UllageVolume; //m^3
    double CurrPressure = 0;
    double UllageMass;
    double OutflowCdA;
    double pressMassFlow = 0;
    double massFlow = 0;
    double outdp = 60;
    double tankVolume = 0.004;
    bool tankEmpty = false;
    ValveState inletValveState = ValveState::Closed;
    ValveState outletValveState = ValveState::Closed;
    ValveState ventValveState = ValveState::Closed;

    void SetValveStates(ValveState InState, ValveState OutState, ValveState VentState);

    void IncompressibleMassFlow(double TimeDelta);

    void pressureUpdateFunction(double TimeDelta, PressurantTank PressTank);

    double getCurrPressure(){return CurrPressure;}//if currpressure stays public don't need get function

    void ChokedMassFlow(double TimeDelta);

    void resetTankObject();

    tankObject(propFluid setTankFluid, double setOutflowCdA);
};



class FluidSystemSimulation
{
    private:
        uint8_t simID;
    public:
        PressurantTank HiPressTank;
        tankObject FuelTank;
        tankObject LoxTank;
        elapsedMicros simTimeEllapsed;
        double TimeDelta = 0.01; //default value for 100Hz
        FluidSystemSimulation(uint8_t setSimID, PressurantTank setHiPressTank, tankObject setFuelTank, tankObject setLoxTank);
    
    //get functions
        uint8_t getSimID(){return simID;}


    // Top Level Fluid Simulation function, run each time step
        void fluidSystemUpdate();
    // Fake analog sensor read function, used for feeding the propulsion controllers simulated data
        float analogRead(uint8_t fakeADCpin);
    // reset simulation to run again
        void resetSim();
    // reset all configurable settings to defaults
        void resetAll();

};



#endif