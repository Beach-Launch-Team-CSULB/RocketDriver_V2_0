#ifndef ENGINECONTROLLERDEFINITIONSRENEGADESF_H
#define ENGINECONTROLLERDEFINITIONSRENEGADESF_H

#include "EngineControllerClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "ValveDefinitionsRenegadeSF.h"
#include "PyroDefinitionsRenegadeSF.h"
#pragma once

#define NUM_ENGINECONTROLLERS 1

EngineController Engine1{5, 2, 250, &FuelMV, &LoxMV, &valve3_3, &EngineIgniter1, &EngineIgniter2, -10000, -1, -1500000, -500000};  //current valve timings are guestimates

//
std::array<EngineController*, NUM_ENGINECONTROLLERS> engineControllerArray{&Engine1};

#endif