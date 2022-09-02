#ifndef SENSORDEFINITIONSRENEGADESF_H
#define SENSORDEFINITIONSRENEGADESF_H

#include "EXTSensorClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "fluidSimulationDefinitions.h"

// define number of sensors here
// Renegade SF
#define NUM_SENSORS 18

// initialize all sensor objects here
// Renegade SF Stand
EXT_SENSOR ThrustMountLoadCell1pos{80, 4, A0, &waterGoesVroom, 2, 10, 100, false};                       // 0
EXT_SENSOR ThrustMountLoadCell1neg{80, 4, A1, &waterGoesVroom, 2, 10, 100, false};                       // 1
EXT_SENSOR ThrustMountLoadCell2pos{81, 4, A2, &waterGoesVroom, 2, 10, 100, false};                       // 2
EXT_SENSOR ThrustMountLoadCell2neg{81, 4, A3, &waterGoesVroom, 2, 10, 100, false};                       // 3
EXT_SENSOR ThrustMountLoadCell3pos{82, 4, A4, &waterGoesVroom, 2, 10, 100, false};                       // 4
EXT_SENSOR ThrustMountLoadCell3neg{82, 4, A5, &waterGoesVroom, 2, 10, 100, false};                       // 5
EXT_SENSOR ChamberPT2{52, 2, ALARA_ANALOG_IN2, &waterGoesVroom, 10, 100, 1000, false, 0.0186, -102.94};                 // 6
EXT_SENSOR ChamberPT1{50, 2, ALARA_ANALOG_IN1, &waterGoesVroom, 10, 100, 1000, false, 0.0185, -128.88};                 // 7
EXT_SENSOR FuelInletPropSidePT{58, 2, ALARA_ANALOG_IN3, &waterGoesVroom, 2, 10, 100, false, 0.0185, -125.74};          // 8
EXT_SENSOR FuelInjectorPT{54, 2, ALARA_ANALOG_IN4, &waterGoesVroom, 10, 100, 500, false, 0.0186, -123.27};             // 9
EXT_SENSOR LoxInletPropSidePT{60, 2, ALARA_ANALOG_IN5, &waterGoesVroom, 2, 10, 100, false, 0.0186, -128.58};           // 10
EXT_SENSOR MVPneumaticsPT{56, 3, ALARA_ANALOG_IN1, &waterGoesVroom, 2, 10, 50, false, 0.0186, -126.56};                // 11
EXT_SENSOR DomeRegFuelPT{79, 3, ALARA_ANALOG_IN2, &waterGoesVroom, 2, 50, 100, false, 0.0186, -126.67};                // 12
EXT_SENSOR DomeRegLoxPT{80, 3, ALARA_ANALOG_IN3, &waterGoesVroom, 2, 50, 100, false, 0.0185, -133.36};                 // 13
EXT_SENSOR FuelTankPT{62, 3, ALARA_ANALOG_IN4, &waterGoesVroom, 2, 10, 100, false, 0.0186, -129.3};                    // 14
EXT_SENSOR LoxTankPT{66, 3, ALARA_ANALOG_IN5, &waterGoesVroom, 2, 10, 100, false, 0.0187, -125.36};                    // 15
EXT_SENSOR HiPressFuelPT{70, 3, ALARA_ANALOG_IN6, &waterGoesVroom, 2, 10, 50, false, 0.0933, -638.38};                 // 16
EXT_SENSOR HiPressLoxPT{72, 3, ALARA_ANALOG_IN7, &waterGoesVroom, 2, 10, 50, false, 0.093, -629.72};                   // 17

/* //FAKESHIT
EXT_SENSOR FakeChamberPT1{150, 2, 41, &waterGoesVroom, simulatedInput};                 // 7
EXT_SENSOR FakeFuelLinePT{158, 2, 32, &waterGoesVroom, simulatedInput};          // 8
EXT_SENSOR FakeLoxLinePT{160, 2, 22, &waterGoesVroom, simulatedInput};           // 10
EXT_SENSOR FakeFuelTankPT{162, 3, 31, &waterGoesVroom, simulatedInput};                    // 14
EXT_SENSOR FakeLoxTankPT{166, 3, 21, &waterGoesVroom, simulatedInput};                    // 15
EXT_SENSOR FakeHiPressPT{170, 3, 11, &waterGoesVroom, simulatedInput};                 // 16
 */
// Sensor Array including Renegade SF only
std::array<EXT_SENSOR*, NUM_SENSORS> sensorArray{&ThrustMountLoadCell1pos, &ThrustMountLoadCell1neg, &ThrustMountLoadCell2pos, &ThrustMountLoadCell2neg,&ThrustMountLoadCell3pos, &ThrustMountLoadCell3neg, &ChamberPT2, &ChamberPT1, &FuelInletPropSidePT, &FuelInjectorPT, &LoxInletPropSidePT, &MVPneumaticsPT, &DomeRegFuelPT, &DomeRegLoxPT, &FuelTankPT, &LoxTankPT, &HiPressFuelPT};

#endif

//EXT_SENSOR ChamberPT1{52, 8, ALARA_ANALOG_IN4, &waterGoesVroom, 10, 100, 800, 0.0185, -128.88};                 // 7
EXT_SENSOR MVPneumaticsPT{56, 7, ALARA_ANALOG_IN4, &waterGoesVroom, 2, 10, 50, 0.0185, -128.88};                // 11
EXT_SENSOR FuelLinePT{58, 8, ALARA_ANALOG_IN5, &waterGoesVroom, 2, 10, 100, 0.0190, -124.86};          // 8
EXT_SENSOR LoxLinePT{60, 8, ALARA_ANALOG_IN6, &waterGoesVroom, 2, 10, 100, 0.0190, -122.76};           // 10
EXT_SENSOR FuelTankPT1{62, 8, ALARA_ANALOG_IN7, &waterGoesVroom, 2, 10, 600, 0.0192, -123.62};                    // 14
EXT_SENSOR FuelTankPT2{64, 8, ALARA_ANALOG_IN8, &waterGoesVroom, 2, 10, 600, 0.0192, -124.96};                    // 14
EXT_SENSOR LoxTankPT1{66, 8, ALARA_ANALOG_IN2, &waterGoesVroom, 2, 10, 600, 0.0193, -122.2};                    // 15
EXT_SENSOR LoxTankPT2{68, 8, ALARA_ANALOG_IN3, &waterGoesVroom, 2, 10, 600, 0.0192, -123.84};                    // 15
EXT_SENSOR HiPressPT{70, 8, ALARA_ANALOG_IN1, &waterGoesVroom, 2, 10, 50, 0.0959, -623.08};                 // 16
