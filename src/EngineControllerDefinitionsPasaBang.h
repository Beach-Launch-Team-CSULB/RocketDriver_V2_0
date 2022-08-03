#ifndef ENGINECONTROLLERDEFINITIONSPASABANG_H
#define ENGINECONTROLLERDEFINITIONSPASABANG_H

#include "EngineControllerClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "ValveDefinitionsPasaBang.h"
#include "PyroDefinitionsPasaBang.h"

#define NUM_ENGINECONTROLLERS 1

/* Valve valve4_1{NormalClosed};
Valve valve4_2{NormalClosed};
Valve valve4_3{NormalClosed};
Pyro pyro3_1{};
Pyro pyro3_2{}; */

EngineController Engine1{4, 8, 350, &LoxMV, &FuelMV, &valve3_3, &EngineIgniter1, &EngineIgniter2, -10000, -1, -1500000, -500000};  //current valve timings are guestimates

//EngineController Engine1{1, 8, &valve3_1, &valve3_2, &valve3_3, &pyro3_1, &pyro3_2, -10000, -1, -1500000, -500000};  //current valve timings are guestimates

//
std::array<EngineController*, NUM_ENGINECONTROLLERS> engineControllerArray{&Engine1};

#endif