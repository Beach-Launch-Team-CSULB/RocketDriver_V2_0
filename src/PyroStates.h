#ifndef PyroStates_H
#define PyroStates_H

// defines the possible pyro states accross all Pyros


enum class PyroState
{
    Off,              // 0
    On,               // 1
    FireCommanded,    // 2
    OnCommanded,      // 3
    OffCommanded,     // 4
    NullReturn,       // 5
    PyroState_SIZE,   // 6
};



#endif