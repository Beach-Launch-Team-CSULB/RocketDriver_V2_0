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
EXT_SENSOR ThrustMountLoadCell1pos{36, 4, A0, &waterGoesVroom, 2, 10, 100, false};                       // 0
EXT_SENSOR ThrustMountLoadCell1neg{38, 4, A1, &waterGoesVroom, 2, 10, 100, false};                       // 1
EXT_SENSOR ThrustMountLoadCell2pos{40, 4, A2, &waterGoesVroom, 2, 10, 100, false};                       // 2
EXT_SENSOR ThrustMountLoadCell2neg{42, 4, A3, &waterGoesVroom, 2, 10, 100, false};                       // 3
EXT_SENSOR ThrustMountLoadCell3pos{44, 4, A4, &waterGoesVroom, 2, 10, 100, false};                       // 4
EXT_SENSOR ThrustMountLoadCell3neg{46, 4, A5, &waterGoesVroom, 2, 10, 100, false};                       // 5
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


ALARAHP_SENSOR RenegadeEngineHP1{121, 2, ALARA_HIGHPOWER_ANALOGREAD1, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP2{122, 2, ALARA_HIGHPOWER_ANALOGREAD2, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP3{123, 2, ALARA_HIGHPOWER_ANALOGREAD3, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP4{124, 2, ALARA_HIGHPOWER_ANALOGREAD4, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP5{125, 2, ALARA_HIGHPOWER_ANALOGREAD5, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP6{126, 2, ALARA_HIGHPOWER_ANALOGREAD6, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP7{127, 2, ALARA_HIGHPOWER_ANALOGREAD7, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP8{128, 2, ALARA_HIGHPOWER_ANALOGREAD8, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP9{129, 2, ALARA_HIGHPOWER_ANALOGREAD9, 0.0006,1.7800};
ALARAHP_SENSOR RenegadeEngineHP10{130, 2, ALARA_HIGHPOWER_ANALOGREAD10, 0.0006,1.7800};

ALARAHP_SENSOR RenegadePropHP1{131, 3, ALARA_HIGHPOWER_ANALOGREAD1, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP2{132, 3, ALARA_HIGHPOWER_ANALOGREAD2, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP3{133, 3, ALARA_HIGHPOWER_ANALOGREAD3, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP4{134, 3, ALARA_HIGHPOWER_ANALOGREAD4, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP5{135, 3, ALARA_HIGHPOWER_ANALOGREAD5, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP6{136, 3, ALARA_HIGHPOWER_ANALOGREAD6, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP7{137, 3, ALARA_HIGHPOWER_ANALOGREAD7, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP8{138, 3, ALARA_HIGHPOWER_ANALOGREAD8, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP9{139, 3, ALARA_HIGHPOWER_ANALOGREAD9, 0.0006,1.7800};
ALARAHP_SENSOR RenegadePropHP10{140, 3, ALARA_HIGHPOWER_ANALOGREAD10, 0.0006,1.7800};

// ALARA Renegade Engine and Prop Node HP sensor array
std::array<SENSORBASE*, 20> HPsensorArray{&RenegadeEngineHP1, &RenegadeEngineHP2, &RenegadeEngineHP3, &RenegadeEngineHP4, &RenegadeEngineHP5, &RenegadeEngineHP6, &RenegadeEngineHP7, &RenegadeEngineHP8, &RenegadeEngineHP9, &RenegadeEngineHP10, &RenegadePropHP1, &RenegadePropHP2, &RenegadePropHP3, &RenegadePropHP4, &RenegadePropHP5, &RenegadePropHP6, &RenegadePropHP7, &RenegadePropHP8, &RenegadePropHP9, &RenegadePropHP10};


// Sensor Array including Renegade SF only
std::array<EXT_SENSOR*, NUM_SENSORS> sensorArray{&ThrustMountLoadCell1pos, &ThrustMountLoadCell1neg, &ThrustMountLoadCell2pos, &ThrustMountLoadCell2neg,&ThrustMountLoadCell3pos, &ThrustMountLoadCell3neg, &ChamberPT2, &ChamberPT1, &FuelInletPropSidePT, &FuelInjectorPT, &LoxInletPropSidePT, &MVPneumaticsPT, &DomeRegFuelPT, &DomeRegLoxPT, &FuelTankPT, &LoxTankPT, &HiPressFuelPT};

#endif