#ifndef FLUIDSIMULATIONDEFINITIONS_H
#define FLUIDSIMULATIONDEFINITIONS_H

#include "fluidSystemSimulation.h"



//tankObject fuelTank{propFluid::denatAlch, 0.00001087}; //orifice cap = 0.00000475, engine = 0.00001087
//tankObject loxTank{propFluid::Lox, 0.00001087};
tankObject fuelTank{propFluid::Water, 0.00000475}; //orifice cap = 0.00000475, engine = 0.00001087
tankObject loxTank{propFluid::Water, 0.00000475};
PressurantTank stupidpaintbollTank{};

FluidSystemSimulation waterGoesVroom{255, stupidpaintbollTank, fuelTank, loxTank};

#endif