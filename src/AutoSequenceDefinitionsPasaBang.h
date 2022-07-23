#ifndef AUTOSEQUENCEDEFINITIONSPASABANG_H
#define AUTOSEQUENCEDEFINITIONSPASABANG_H

#include "AutoSequenceClass.h"
#include <array>

// Define the number of autosequences here
#define NUM_AUTOSEQUENCES 1


AutoSequence IgnitionAutoSequence{-15000000, 8}; // X seconds in micros to set the countdownStart

std::array<AutoSequence*, NUM_AUTOSEQUENCES> autoSequenceArray{&IgnitionAutoSequence};


#endif