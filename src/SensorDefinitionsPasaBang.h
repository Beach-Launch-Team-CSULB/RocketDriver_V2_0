#ifndef SENSORDEFINITIONSPASABANG_H
#define SENSORDEFINITIONSPASABANG_H

#include "ALARAHPSensorClass.h"
#include "ALARAVRailSensorClass.h"
#include "EXTSensorClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "fluidSimulationDefinitions.h"

// define number of sensors here
// Renegade SF
#define NUM_SENSORS 15
#define NUM_HPSENSORS 12

// initialize all sensor objects here
// PasaFire BangBang Config
//EXT_SENSOR ChamberPT1{52, 8, ALARA_ANALOG_IN4, &waterGoesVroom, 10, 100, 800, 0.0185, -128.88};                 // 7
EXT_SENSOR MVPneumaticsPT{56, 7, ALARA_ANALOG_IN4, &waterGoesVroom, 2, 10, 50, 0.0185, -128.88};                // 11
EXT_SENSOR FuelLinePT{58, 8, ALARA_ANALOG_IN5, &waterGoesVroom, 2, 10, 100, 0.0190, -124.86};          // 8
EXT_SENSOR LoxLinePT{60, 8, ALARA_ANALOG_IN6, &waterGoesVroom, 2, 10, 100, 0.0190, -122.76};           // 10
EXT_SENSOR FuelTankPT1{62, 8, ALARA_ANALOG_IN7, &waterGoesVroom, 2, 10, 600, 0.0192, -123.62};                    // 14
EXT_SENSOR FuelTankPT2{64, 8, ALARA_ANALOG_IN8, &waterGoesVroom, 2, 10, 600, 0.0192, -124.96};                    // 14
EXT_SENSOR LoxTankPT1{66, 8, ALARA_ANALOG_IN2, &waterGoesVroom, 2, 10, 600, 0.0193, -122.2};                    // 15
EXT_SENSOR LoxTankPT2{68, 8, ALARA_ANALOG_IN3, &waterGoesVroom, 2, 10, 600, 0.0192, -123.84};                    // 15
EXT_SENSOR HiPressPT{70, 8, ALARA_ANALOG_IN1, &waterGoesVroom, 2, 10, 50, 0.0959, -623.08};                 // 16

//FAKESHIT
//EXT_SENSOR FakeChamberPT1{156, 8, 41, &waterGoesVroom, simulatedInput};                 // 7
EXT_SENSOR FakeFuelLinePT{158, 8, 32, &waterGoesVroom, simulatedInput};          // 8
EXT_SENSOR FakeLoxLinePT{160, 8, 22, &waterGoesVroom, simulatedInput};           // 10
EXT_SENSOR FakeFuelTankPT{162, 8, 31, &waterGoesVroom, simulatedInput};                    // 14
EXT_SENSOR FakeLoxTankPT{166, 8, 21, &waterGoesVroom, simulatedInput};                    // 15
EXT_SENSOR FakeHiPressPT{170, 8, 11, &waterGoesVroom, simulatedInput};                 // 16

EXT_SENSOR ThrustMountLoadCell1pos{76, 7, A3, &waterGoesVroom, 2, 10, 100, false};                       // 0
EXT_SENSOR ThrustMountLoadCell1neg{78, 7, A2, &waterGoesVroom, 2, 10, 100, false};                       // 1

ALARAHP_SENSOR PasafireHP1{81, 8, ALARA_HIGHPOWER_ANALOGREAD1, 0.0006,1.7951};
ALARAHP_SENSOR PasafireHP2{82, 8, ALARA_HIGHPOWER_ANALOGREAD2, 0.0006,1.7957};
ALARAHP_SENSOR PasafireHP3{83, 8, ALARA_HIGHPOWER_ANALOGREAD3, 0.0006,1.7834};
ALARAHP_SENSOR PasafireHP4{84, 8, ALARA_HIGHPOWER_ANALOGREAD4, 0.0006,1.7744};
ALARAHP_SENSOR PasafireHP5{85, 8, ALARA_HIGHPOWER_ANALOGREAD5, 0.0006,1.7703};
ALARAHP_SENSOR PasafireHP6{86, 8, ALARA_HIGHPOWER_ANALOGREAD6, 0.0006,1.8048};
ALARAHP_SENSOR PasafireHP7{87, 8, ALARA_HIGHPOWER_ANALOGREAD7, 0.0006,1.7779};
ALARAHP_SENSOR PasafireHP8{88, 8, ALARA_HIGHPOWER_ANALOGREAD8, 0.0006,1.7862};
ALARAHP_SENSOR PasafireHP9{89, 8, ALARA_HIGHPOWER_ANALOGREAD9, 0.0006,1.7831};
ALARAHP_SENSOR PasafireHP10{90, 8, ALARA_HIGHPOWER_ANALOGREAD10, 0.0006,1.7689};

//MCU_INTERNAL TEMP PasafireMCUTemp{212, 8};
ALARAVRAIL_SENSOR Pasafire3_3{214, 8, ALARA_ANALOG_3_3RAIL, 0.0006,1.7800}; // not calibrated
ALARAVRAIL_SENSOR Pasafire5_0{216, 8, ALARA_ANALOG_5RAIL, 0.0006,1.7800};   // not calibrated


/* ALARAHP_SENSOR RenegadeEngineHP1{121, 2, ALARA_HIGHPOWER_ANALOGREAD1, 0.0006,1.7800};
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
ALARAHP_SENSOR RenegadePropHP10{140, 3, ALARA_HIGHPOWER_ANALOGREAD10, 0.0006,1.7800}; */

// Sensor Array including PasaBang only
//std::array<MCU_SENSOR*, NUM_SENSORS> sensorArray{&ChamberPT1, &FuelLinePT, &LoxLinePT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT};
std::array<SENSORBASE*, NUM_SENSORS> sensorArray{&MVPneumaticsPT, &FuelLinePT, &LoxLinePT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT, &FakeFuelLinePT, &FakeLoxLinePT, &FakeFuelTankPT, &FakeLoxTankPT, &FakeHiPressPT, &ThrustMountLoadCell1pos, &ThrustMountLoadCell1neg};

// ALARA Pasafire HP sensor array
std::array<SENSORBASE*, NUM_HPSENSORS> HPsensorArray{&PasafireHP1, &PasafireHP2, &PasafireHP3, &PasafireHP4, &PasafireHP5, &PasafireHP6, &PasafireHP7, &PasafireHP8, &PasafireHP9, &PasafireHP10, &Pasafire3_3, &Pasafire5_0};

/* // ALARA Renegade Engine and Prop Node HP sensor array
std::array<SENSORBASE*, 20> HPsensorArray{&RenegadeEngineHP1, &RenegadeEngineHP2, &RenegadeEngineHP3, &RenegadeEngineHP4, &RenegadeEngineHP5, &RenegadeEngineHP6, &RenegadeEngineHP7, &RenegadeEngineHP8, &RenegadeEngineHP9, &RenegadeEngineHP10, &RenegadePropHP1, &RenegadePropHP2, &RenegadePropHP3, &RenegadePropHP4, &RenegadePropHP5, &RenegadePropHP6, &RenegadePropHP7, &RenegadePropHP8, &RenegadePropHP9, &RenegadePropHP10};
 */

#endif