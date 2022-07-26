#include "fluidSystemSimulation.h"

bool ISTHISREAL = false;  //flag for test mode vs driving real system. False means test, True means real
bool ISVALVEONLYREAL = false;
bool serialStreamingLog = false;
bool livePlotOutputOnly = false;
elapsedMillis tankPressDelayTimer = 0;
uint32_t tankPressDelay = 10000;
//bool MVOpen = false;


elapsedMillis bangMVtimer = 0;
//ValveState bang1ValveState = ValveState::Closed;
//ValveState bang2ValveState = ValveState::Closed;

//float loopyboi = 0;
//float controllerTargetValue = 150;
//int controllerTargetValueInt = static_cast<int>(controllerTargetValue+0.5);
//IntervalTimer pressureUpdateInterval;
//IntervalTimer banbBangcontrollerInterval;
//uint32_t controllerIntervalHz = 200;
//uint32_t sensorIntervalHz = 800;
//float controllerTimeStep = 1/controllerIntervalHz;
//float sensorIntervalTimeStep = 1/sensorIntervalHz;
//float controllerTimeStep = 1/static_cast<float>(controllerIntervalHz);
//float sensorIntervalTimeStep = 1/static_cast<float>(sensorIntervalHz);

////////////
//float TankVolume = 0.1; //m^3
//float OptimalTankPress = controllerTargetValue* unitConversionCosnt;
float orificeOutDiameter = 0.1*0.0254; //m
float orificeOutArea = ((orificeOutDiameter*orificeOutDiameter)*M_PI)/4;
float orificeInDiameter = 0.1*0.0254; //m
float orificeInArea = ((orificeInDiameter*orificeInDiameter)*M_PI)/4;
float COPVPress = 4000* unitConversionCosnt; //psi
//float Time = 0;
//float TimeDelta = sensorIntervalTimeStep; //artist formerly known as 0.01
//float TankMass = 0;
//float CurrPressure = 0;
elapsedMillis valveTimer;
//float gasDensity = 0;
//float TankPropMass = 0;
//float TankPressure = 0;
float massFlow = 0;
//float accumulatedIntegral = 0;

////////


float tankObject::CurrTankPress(float TankPropMass)
{
  gasDensity = TankPropMass/TankVolume;
  TankPressure = gasDensity*AirGasConstant*ATPtemp;
/*   Serial.print("Pizza TankPropMass: ");
  Serial.println(TankPropMass);
  Serial.print("Pizza TankVolume: ");
  Serial.println(TankVolume);
  Serial.print("Pizza gasDensity: ");
  Serial.println(gasDensity);
  Serial.print("Pizza TankPressure: ");
  Serial.println(TankPressure); */
  return TankPressure;
}

float ChokedMassFlow(float UpstreamPressure, float chockedOrificeArea)
{
  massFlow = Cd*UpstreamPressure*chockedOrificeArea*sqrt((Gamma/(AirGasConstant*ATPtemp))*pow((2/(Gamma+1)), ((Gamma+1)/(Gamma-1))));
  return massFlow;
}

//for interrupt timers, not sure how to setup to run the other stuff yet. Need to pass everything into this, then into other functions inside?
// it should work fine, anything that needs to be updated to pass into the functions will pass through every time the interrupt runs (I think)
void tankObject::pressureUpdateFunction(float TimeDelta)
{  
if (!(outletValveState == ValveState::Closed))
{
  if (inletValveState == ValveState::Open || inletValveState == ValveState::BangingOpen)
  {
  TankMass += ChokedMassFlow(COPVPress,orificeInArea)*TimeDelta;
  }
  if (inletValveState == ValveState::Closed || inletValveState == ValveState::BangingClosed)
  {
    //nothing "leaking"
  }
}
else
{
  if (inletValveState == ValveState::Open || inletValveState == ValveState::BangingOpen)
  {
  TankMass += ChokedMassFlow(COPVPress,orificeInArea)*TimeDelta;
  TankMass -= ChokedMassFlow(CurrPressure*unitConversionCosnt,orificeOutArea)*TimeDelta;
  }
  if (inletValveState == ValveState::Closed || inletValveState == ValveState::BangingClosed)
  {
  //TankMass += ChokedMassFlow(COPVPress,orificeInArea)*TimeDelta;
  TankMass -= ChokedMassFlow(CurrPressure*unitConversionCosnt,orificeOutArea)*TimeDelta;
  }
}
  CurrPressure = CurrTankPress(TankMass)/unitConversionCosnt;
}


    //analog PT read stuff here to use real pressure value
/*     currentRawValue1 = adc->analogRead(BANGFUELPT_ANALOGINPUT);
    currentConvertedValue1 = fueltankPT_linConvCoef1_m*currentRawValue1 + fueltankPT_linConvCoef1_b;
    loopyboi = currentConvertedValue1;
    loopyboi2 = static_cast<int>(currentConvertedValue1+0.5); */

  
