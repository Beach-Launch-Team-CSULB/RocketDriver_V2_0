#ifndef PYRODEFINITIONSRENEGADESF_H
#define PYRODEFINITIONSRENEGADESF_H

#include "PyroClass.h"
#include <array>
#include "ALARApinDefines.h"
#pragma once

// Define number of pyros here
#define NUM_PYROS 2

// Declare all Pyro Objects here using PyroClass, and add them to the pyroArray

// NEED TO FIX PIN MAPPING FOR THE SECOND OUTPUT TO NEW REAL STAND WIRING
// Engine Node
Pyro EngineIgniter1{26, 2, 5, 2000000};   // 0
Pyro EngineIgniter2{27, 2, 7, 500000};    // 1

// ADD PYROS TO THIS VALVE ARRAY IN THE FORM: &PYRO
std::array<Pyro*, NUM_PYROS> pyroArray{&EngineIgniter1, &EngineIgniter2};

#endif