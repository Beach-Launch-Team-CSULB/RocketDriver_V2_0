#ifndef PIEZOSONGS_H
#define PIEZOSONGS_H
#include "vector"
#include "Arduino.h"
using std::vector;

struct buzzerTone
{
    float toneLoudness;       // Loudness as a % of the buzzer range minimum audible = 0%, max volume = 100%
    float toneFrequency;            // Tone Frequency in Hz
    uint32_t toneDuration;          // Length in micros of tone before rest or next tone
};

//empty default songVector of buzzerTone structs
//std::vector<buzzerTone> songVector{};
// song vector generator functions
std::vector<buzzerTone> songTheImperialMarch();
std::vector<buzzerTone> songTheMandalorianTheme();

#endif