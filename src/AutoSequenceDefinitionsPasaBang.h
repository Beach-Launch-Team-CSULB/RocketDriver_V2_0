#ifndef AUTOSEQUENCEDEFINITIONSPASABANG_H
#define AUTOSEQUENCEDEFINITIONSPASABANG_H

#include "AutoSequenceClass.h"
#include <array>

// Define the number of autosequences here
#define NUM_AUTOSEQUENCES 1

// ID, X seconds [signed value T- format!!!] in micros to set the countdownStart, host node ID
AutoSequence IgnitionAutoSequence{1,-5000000, 8}; 

std::array<AutoSequence*, NUM_AUTOSEQUENCES> autoSequenceArray{&IgnitionAutoSequence};


#endif