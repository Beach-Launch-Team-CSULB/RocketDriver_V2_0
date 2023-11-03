#ifndef ENGINECONTROLLERDEFINITIONS_H
#define ENGINECONTROLLERDEFINITIONS_H

#include "EngineControllerClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "ValveDefinitions.h"
#include "PyroDefinitions.h"
#pragma once

#ifdef PASABANG
#define NUM_ENGINECONTROLLERS 1
EngineController Engine1{5, 8, 250, &FuelMV, &LoxMV, &valve3_3, &EngineIgniter1, &EngineIgniter2, -10000, -1, -1000000, -300000};  //current valve timings are guestimates
//
std::array<EngineController*, NUM_ENGINECONTROLLERS> engineControllerArray{&Engine1};
#endif

#ifdef RENEGADESF
#define NUM_ENGINECONTROLLERS 1
// fake lazy way to fill valve not in use ayy
Valve valve3_3{NormalClosed};
EngineController Engine1{5, 2, 250, &FuelMV, &LoxMV, &valve3_3, &EngineIgniter1, &EngineIgniter2, -200000, 0, -3000000, -3000000};  //current valve timings are guestimates
//
std::array<EngineController*, NUM_ENGINECONTROLLERS> engineControllerArray{&Engine1};
#endif

#endif