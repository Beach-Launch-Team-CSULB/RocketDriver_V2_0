#ifndef SENSORDEFINITIONSPASABANG_H
#define SENSORDEFINITIONSPASABANG_H

#include "SensorClass.h"
#include <array>
#include "ALARApinDefines.h"

// define number of sensors here
// Renegade SF
#define NUM_SENSORS 12

// initialize all sensor objects here
// PasaFire BangBang Config
MCU_SENSOR ThrustMountLoadCell1pos{50, 2, A0, 2, 10, 100, false};                       // 0
MCU_SENSOR ThrustMountLoadCell1neg{50, 5, A0, 2, 10, 100, false};                       // 1
MCU_SENSOR ChamberPT1{56, 2, A6, 10, 100, 800, false, 0.0185, -128.88};                 // 7
MCU_SENSOR FuelLinePT{57, 2, A7, 2, 10, 100, false, 0.0185, -125.74};          // 8
MCU_SENSOR LoxLinePT{59, 2, A9, 2, 10, 100, false, 0.0186, -128.58};           // 10
MCU_SENSOR MVPneumaticsPT{78, 3, A3, 2, 10, 50, false, 0.0186, -126.56};                // 11
MCU_SENSOR FuelTankPT1{81, 3, A6, 2, 10, 100, false, 0.0186, -129.3};                    // 14
MCU_SENSOR FuelTankPT2{81, 3, A6, 2, 10, 100, false, 0.0186, -129.3};                    // 14
MCU_SENSOR LoxTankPT1{82, 3, A7, 2, 10, 100, false, 0.0187, -125.36};                    // 15
MCU_SENSOR LoxTankPT2{82, 3, A7, 2, 10, 100, false, 0.0187, -125.36};                    // 15
MCU_SENSOR HiPressPT{83, 3, A8, 2, 10, 50, false, 0.0933, -638.38};                 // 16
MCU_SENSOR MCUtempNode8{200, 8, 70, 2, 5, 10, true};                                    // 18

// Sensor Array including Renegade SF only
std::array<MCU_SENSOR*, NUM_SENSORS> sensorArray{&ThrustMountLoadCell1pos, &ThrustMountLoadCell1neg, &ChamberPT1, &FuelLinePT, &LoxLinePT, &MVPneumaticsPT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT, &MCUtempNode8};

#endif