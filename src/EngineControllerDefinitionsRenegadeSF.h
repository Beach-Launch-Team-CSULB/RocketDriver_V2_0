#ifndef ENGINECONTROLLERDEFINITIONSRENEGADESF_H
#define ENGINECONTROLLERDEFINITIONSRENEGADESF_H

#include "EngineControllerClass.h"
#include <array>
#include "ALARApinDefines.h"

#define NUM_ENGINECONTROLLERS 1

EngineController Engine1{1, 2, -10000, -1, -1500000, -500000};  //current valve timings are guestimates

//
std::array<EngineController*, NUM_ENGINECONTROLLERS> engineControllerArray{&Engine1};

#endif