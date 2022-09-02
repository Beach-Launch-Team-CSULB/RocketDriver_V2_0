#ifndef SENSORDEFINITIONSPASABANG_H
#define SENSORDEFINITIONSPASABANG_H

#include "EXTSensorClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "fluidSimulationDefinitions.h"

// define number of sensors here
// Renegade SF
#define NUM_SENSORS 15

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


// Sensor Array including PasaBang only
//std::array<MCU_SENSOR*, NUM_SENSORS> sensorArray{&ChamberPT1, &FuelLinePT, &LoxLinePT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT};
std::array<SENSORBASE*, NUM_SENSORS> sensorArray{&MVPneumaticsPT, &FuelLinePT, &LoxLinePT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT, &FakeFuelLinePT, &FakeLoxLinePT, &FakeFuelTankPT, &FakeLoxTankPT, &FakeHiPressPT, &ThrustMountLoadCell1pos, &ThrustMountLoadCell1neg};

#endif