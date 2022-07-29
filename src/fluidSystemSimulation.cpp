#include "fluidSystemSimulation.h"

int unitConversionCosnt = 6895;
float ATPtemp = 288.15; //K
float PropDensity = 999;
float N2GasConst = 296.8; //J/kg-k
float Gamma = 1.4;



bool ISTHISREAL = false;  //flag for test mode vs driving real system. False means test, True means real
bool ISVALVEONLYREAL = false;
bool serialStreamingLog = false;
bool livePlotOutputOnly = false;
elapsedMillis tankPressDelayTimer = 0;
uint32_t tankPressDelay = 10000;


elapsedMillis bangMVtimer = 0;

////////////

float orificeOutDiameter = 0.1*0.0254; //m
float orificeOutArea = ((orificeOutDiameter*orificeOutDiameter)*M_PI)/4;
float orificeInDiameter = 0.1*0.0254; //m
float orificeInArea = ((orificeInDiameter*orificeInDiameter)*M_PI)/4;

elapsedMillis valveTimer;


////////



FluidSystemSimulation::FluidSystemSimulation(float setTimeDelta, PressurantTank setHiPressTank, tankObject setFuelTank, tankObject setLoxTank):TimeDelta{setTimeDelta}, HiPressTank{setHiPressTank}, FuelTank{setFuelTank}, LoxTank{setLoxTank} 
{
  
}

tankObject::tankObject(float setOutflowCdA): OutflowCdA{setOutflowCdA}
{ 
    UllageMass = CurrPressure / N2GasConst / ATPtemp * UllageVolume; //placeholder, probly dumb
}

PressurantTank::PressurantTank()
{
  PressurantMass = CurrPressure / N2GasConst / ATPtemp * TankVolume;
}

void tankObject::IncompressibleMassFlow(float TimeDelta)
{
  float massFlow = OutflowCdA*sqrt(2*CurrPressure*PropDensity);
  UllageVolume += massFlow/PropDensity*TimeDelta;
  CurrPressure = UllageMass / UllageVolume *N2GasConst*ATPtemp;
}

float PressurantTank::ChokedMassFlow(float TimeDelta)
{
    float massFlow = CdA*CurrPressure*sqrt((Gamma/(N2GasConst*ATPtemp))*pow((2/(Gamma+1)), ((Gamma+1)/(Gamma-1))));
    PressurantMass -= TimeDelta*massFlow;
    CurrPressure = PressurantMass / TankVolume *N2GasConst*ATPtemp;
    return massFlow;
}

void tankObject::SetValveStates(ValveState InState, ValveState OutState, ValveState VentState)
{
  inletValveState = valveStateFlowSimSimplify(InState);
  outletValveState = valveStateFlowSimSimplify(OutState);
  ventValveState = valveStateFlowSimSimplify(VentState);
/* Serial.print("sim set valve states");
Serial.print(static_cast<uint8_t>(inletValveState));
Serial.print(static_cast<uint8_t>(outletValveState));
Serial.println(static_cast<uint8_t>(ventValveState)); */


};



//for interrupt timers, not sure how to setup to run the other stuff yet. Need to pass everything into this, then into other functions inside?
// it should work fine, anything that needs to be updated to pass into the functions will pass through every time the interrupt runs (I think)
void tankObject::pressureUpdateFunction(float TimeDelta, PressurantTank PressTank)
{
Serial.print("valve states inside update func");
Serial.print(" : inlet");
Serial.print(static_cast<uint8_t>(inletValveState));
Serial.print(" : outlet");
Serial.print(static_cast<uint8_t>(outletValveState));
Serial.print(" : vent : ");
Serial.println(static_cast<uint8_t>(ventValveState));
if (outletValveState == ValveState::Closed && inletValveState == ValveState::Open)
{
    Serial.print("supposed to be Tank Press");
    float pressMassFlow = PressTank.ChokedMassFlow(TimeDelta);
    UllageMass += pressMassFlow*TimeDelta;
    CurrPressure = UllageMass / UllageVolume *N2GasConst*ATPtemp;
}
else if (outletValveState == ValveState::Open && inletValveState == ValveState::Open)
{
    Serial.print("supposed to be MV open");
    float pressMassFlow = PressTank.ChokedMassFlow(TimeDelta);
    UllageMass += pressMassFlow*TimeDelta;
    IncompressibleMassFlow(TimeDelta);
    
  }
else if (outletValveState == ValveState::Open && inletValveState == ValveState::Closed)
{
    Serial.print("supposed nuttin");
    IncompressibleMassFlow(TimeDelta);
}
    Serial.print("N/A");
}


ValveState valveStateFlowSimSimplify(ValveState inputValveState)
{
  // this function takes a given valve object state and returns a binary flow state of open or closed
  if (inputValveState == ValveState::BangingOpen || inputValveState == ValveState::BangOpenProcess || inputValveState == ValveState::OpenProcess || inputValveState == ValveState::Open)
  {
    inputValveState = ValveState::Open;
  }
  else
  {
    inputValveState = ValveState::Closed;
  }
  return inputValveState;
}
  
void FluidSystemSimulation::fluidSystemUpdate(){
  // PID SHIT GOES HERE

  //Sets the state of the valves
  //FuelTank.SetValveStates(ValveState::Closed,ValveState::Closed,ValveState::Closed);
  //LoxTank.SetValveStates(ValveState::Closed,ValveState::Closed,ValveState::Closed);


  // AFTER PID CHOOSES VALVE STATE FLOW HAPPENS AND PRESSURE CHANGES
  FuelTank.pressureUpdateFunction(TimeDelta, HiPressTank);
  LoxTank.pressureUpdateFunction(TimeDelta, HiPressTank);
  Serial.println();
  Serial.print(FuelTank.CurrPressure);
  Serial.print(" : ");
  Serial.print(LoxTank.CurrPressure);
  Serial.print(" : ");
  Serial.print(HiPressTank.CurrPressure);
  Serial.println(" fluid sim update ran");
}

float FluidSystemSimulation::analogRead(uint8_t fakeADCpin)
{
  float simulatedSensorReading = 0;
  if (fakeADCpin == 31)
  {
    simulatedSensorReading = FuelTank.CurrPressure;
  }
  else if (fakeADCpin == 21)
  {
    simulatedSensorReading = LoxTank.CurrPressure;
  }
  else if (fakeADCpin == 11)
  {
    simulatedSensorReading = HiPressTank.CurrPressure;
  }
  
/*   Serial.print(" simulatedSensor fake pin : ");
  Serial.println(fakeADCpin);
  Serial.print(" simulatedSensorReading : ");
  Serial.println(simulatedSensorReading); */
  return simulatedSensorReading;
}
