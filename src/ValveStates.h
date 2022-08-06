#ifndef VALVESTATES_H
#define VALVESTATES_H

// defines the possible valve states accross all valves

enum class ValveState
{
    Closed,             // 0
    Open,               // 1
    FireCommanded,      // 2
    OpenCommanded,      // 3
    CloseCommanded,     // 4
    OpenProcess,        // 5
    CloseProcess,       // 6
    BangOpenCommanded,  // 7
    BangCloseCommanded, // 8
    BangOpenProcess,    // 9
    BangCloseProcess,   // 10
    BangingOpen,        // 11
    BangingClosed,      // 12
    NullReturn,         // 13
    ValveState_SIZE,    // 14
};

// detected states based on measuring the ADC channel on ALARA HP outputs
enum class ValveElectricalOutputState
{
    Off,
    On,
    NoContinuity,
};

#endif