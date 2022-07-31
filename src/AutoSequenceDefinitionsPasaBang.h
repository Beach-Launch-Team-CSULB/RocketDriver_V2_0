#ifndef AUTOSEQUENCEDEFINITIONSPASABANG_H
#define AUTOSEQUENCEDEFINITIONSPASABANG_H

#include "AutoSequenceClass.h"
#include <array>

// Define the number of autosequences here
#define NUM_AUTOSEQUENCES 1

// ID, X seconds in micros to set the countdownStart, host node ID
AutoSequence IgnitionAutoSequence{0,-15000000, 8}; 

std::array<AutoSequence*, NUM_AUTOSEQUENCES> autoSequenceArray{&IgnitionAutoSequence};


#endif