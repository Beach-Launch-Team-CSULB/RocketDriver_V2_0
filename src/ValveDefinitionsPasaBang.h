#ifndef VALVEDEFINITIONSPASABANG_H
#define VALVEDEFINITIONSPASABANG_H

#include "ValveClass.h"
#include <array>
#include "ALARApinDefines.h"

// Define number of valves here
#define NUM_VALVES 7

// Declare all Valve Objects here using ValveClass, and add them to the valveArray
// On Pasafire Node
Valve HiPressVent{17, 8, NormalClosed, 5, 24, 25000, false};
Valve LoxMV{24, 8, NormalClosed, 7, 26, 25000, false, ValveState::CloseCommanded, 50};
Valve FuelMV{25, 8, NormalClosed, 6, 26, 25000, false, ValveState::CloseCommanded, 50};
Valve LoxVent{18, 8, NormalOpen, 2, 27, 500000, false, ValveState::CloseCommanded, 166}; // don't actually want the abort state here but need to change constructor
Valve LoxBang{19, 8, NormalClosed, 5, 25, 25000, false};
Valve FuelVent{21, 8, NormalClosed, 9, 27, 25000, false};
Valve FuelBang{22, 8, NormalClosed, 7, 26, 25000, false};

// ADD VALVES TO THIS VALVE ARRAY IN THE FORM: &VALVE
std::array<Valve*, NUM_VALVES> valveArray{&HiPressVent, &LoxMV, &FuelMV, &LoxVent, &LoxBang, &FuelVent, &FuelBang};

#endif