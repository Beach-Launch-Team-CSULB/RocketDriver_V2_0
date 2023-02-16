#ifndef AUTOSEQUENCEDEFINITIONS_H
#define AUTOSEQUENCEDEFINITIONS_H

#include "AutoSequenceClass.h"
#include <array>

#ifdef RENEGADESF
// Define the number of autosequences here
#define NUM_AUTOSEQUENCES 1
// ID, X seconds [signed value T- format!!!] in micros to set the countdownStart, host node ID
AutoSequence IgnitionAutoSequence{1,-15000000, 2};
//
std::array<AutoSequence*, NUM_AUTOSEQUENCES> autoSequenceArray{&IgnitionAutoSequence};
#endif

#endif