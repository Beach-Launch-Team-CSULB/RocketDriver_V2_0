#ifndef FLUIDSIMULATIONDEFINITIONS_H
#define FLUIDSIMULATIONDEFINITIONS_H

#include "fluidSystemSimulation.h"



tankObject fuelTank{0.01};
tankObject loxTank{0.01};
PressurantTank stupidpaintbollTank{};

FluidSystemSimulation waterGoesVroom{0.001,stupidpaintbollTank,fuelTank,loxTank};

#endif