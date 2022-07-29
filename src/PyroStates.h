#ifndef PyroStates_H
#define PyroStates_H

// defines the possible pyro states accross all Pyros


enum class PyroState
{
    Off,                // 0
    On,                 // 1
    FireCommanded,      // 2
    OnCommanded,        // 3
    OffCommanded,       // 4
    Fired,              // 5
    NullReturn,         // 6
    PyroState_SIZE,     // 7
};



#endif