#ifndef PYRODEFINITIONSPASABANG_H
#define PYRODEFINITIONSPASABANG_H

#include "PyroClass.h"
#include <array>
#include "ALARApinDefines.h"
#pragma once

// Define number of pyros here
#define NUM_PYROS 2

// Declare all Pyro Objects here using PyroClass, and add them to the pyroArray

// NEED TO FIX PIN MAPPING FOR THE SECOND OUTPUT TO NEW REAL STAND WIRING
// Engine Node
Pyro EngineIgniter1{26, 8, 9, 2000000};   // 0
Pyro EngineIgniter2{27, 8, 10, 2000000};    // 1

/* // Engine Node
Pyro EngineIgniter1{26, 8, ALARA_HIGHPOWER_PWMOUT9, ALARA_HIGHPOWER_DIGITALOUT9, 2000000};   // 0
Pyro EngineIgniter2{27, 8, ALARA_HIGHPOWER_PWMOUT10, ALARA_HIGHPOWER_DIGITALOUT10, 2000000};    // 1 */

// ADD PYROS TO THIS VALVE ARRAY IN THE FORM: &PYRO
std::array<Pyro*, NUM_PYROS> pyroArray{&EngineIgniter1, &EngineIgniter2};

#endif