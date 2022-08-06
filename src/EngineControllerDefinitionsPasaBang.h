#ifndef ENGINECONTROLLERDEFINITIONSPASABANG_H
#define ENGINECONTROLLERDEFINITIONSPASABANG_H

#include "EngineControllerClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "ValveDefinitionsPasaBang.h"
#include "PyroDefinitionsPasaBang.h"

#define NUM_ENGINECONTROLLERS 1

EngineController Engine1{5, 8, 250, &LoxMV, &FuelMV, &valve3_3, &EngineIgniter1, &EngineIgniter2, -10000, -1, -1500000, -500000};  //current valve timings are guestimates

//
std::array<EngineController*, NUM_ENGINECONTROLLERS> engineControllerArray{&Engine1};

#endif