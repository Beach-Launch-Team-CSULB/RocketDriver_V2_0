#ifndef VALVEDEFINITIONS_H
#define VALVEDEFINITIONS_H

#include "ValveClass.h"
#include <array>
#include "ALARApinDefines.h"
#pragma once

#ifdef RENEGADESF
// Define number of valves here
#define NUM_VALVES 10
// Declare all Valve Objects here using ValveClass, and add them to the valveArray
// On Renegade SF Engine Node
Valve HiPress{16, 2, NormalClosed, 1, 25000, false};
Valve HiPressVent{17, 2, NormalClosed, 2, 25000, false};
Valve LoxMV{24, 2, NormalClosed, 4, 25000, false, 50};
Valve FuelMV{25, 2, NormalClosed, 3, 25000, false, 50};
// On Renegade SF Prop Node
Valve LoxVent{18, 3, NormalOpen, 1, 500000, false, 50};
Valve LoxDomeReg{19, 3, NormalClosed, 3, 25000, false};
Valve LoxDomeRegVent{20, 3, NormalClosed, 4, 25000, false};
Valve FuelVent{21, 3, NormalClosed, 5, 25000, false};
Valve FuelDomeReg{22, 3, NormalClosed, 7, 25000, false};
Valve FuelDomeRegVent{23, 3, NormalClosed, 8, 25000, false};
// ADD VALVES TO THIS VALVE ARRAY IN THE FORM: &VALVE
std::array<Valve*, NUM_VALVES> valveArray{&HiPress, &HiPressVent, &LoxMV, &FuelMV, &LoxVent, &LoxDomeReg, &LoxDomeRegVent, &FuelVent, &FuelDomeReg, &FuelDomeRegVent};
#endif

#endif