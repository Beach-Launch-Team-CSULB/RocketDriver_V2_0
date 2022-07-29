#ifndef FLUIDSIMULATIONDEFINITIONS_H
#define FLUIDSIMULATIONDEFINITIONS_H

#include "fluidSystemSimulation.h"



tankObject fuelTank{.00000151};
tankObject loxTank{.00000151};
PressurantTank stupidpaintbollTank{};

FluidSystemSimulation waterGoesVroom{.01,stupidpaintbollTank,fuelTank,loxTank};

#endif