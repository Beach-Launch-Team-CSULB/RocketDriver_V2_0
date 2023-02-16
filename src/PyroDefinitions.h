#ifndef PYRODEFINITIONS_H
#define PYRODEFINITIONS_H

#include "PyroClass.h"
#include <array>
#include "ALARApinDefines.h"
#pragma once

#ifdef RENEGADESF
// Define number of pyros here
#define NUM_PYROS 2
// Declare all Pyro Objects here using PyroClass, and add them to the pyroArray
// Engine Node
Pyro EngineIgniter1{26, 2, 5, 500000};   // 0
Pyro EngineIgniter2{27, 2, 7, 500000};    // 1
// ADD PYROS TO THIS VALVE ARRAY IN THE FORM: &PYRO
std::array<Pyro*, NUM_PYROS> pyroArray{&EngineIgniter1, &EngineIgniter2};

#endif
#endif