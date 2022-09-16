#ifndef CONTROLLERSTATES_H
#define CONTROLLERSTATES_H

// defines the possible controller states accross propulsion controllers
// very preliminary, update as needed while developing controllers
// Current plan - setup different controller types as different state enum lists

enum class TankPressControllerState
{
    Passive,                    // 0
    Standby,                    // 1
    BangBangActive,             // 2
    RegPressActive,            // 3
    HiPressPassthroughVent,     // 4
    Armed,                      // 5
    PropTankVent,               // 6
    HiVent,                     // 7
    Abort,                      // 8
    TestPassthrough,            // 9
    OffNominalPassthrough,      // 10
    AutosequenceCommanded,      // 11
    ControllerState_SIZE,       // 12
};

// generic below, probably supercede/replace as I get to other controllers
enum class ControllerState
{
    Passive,                    // 0
    Active,                     // 1
    Armed,                      // 2
    Vent,                       // 3
    Abort,                      // 4
    TestPassthrough,            // 5
    AutosequenceCommanded,      // 6
    ControllerState_SIZE,       // 7
};


enum class EngineControllerState
{
    Passive,                    // 0
    Active,                     // 1
    Chill,                      // 2
    Purge,                      // 3
    Shutdown,                   // 4
    Armed,                      // 5
    TestPassthrough,            // 6
    OffNominalPassthrough,      // 7
    FiringAutosequence,         // 8
    ControllerState_SIZE,       // 9
};

enum class ALARAV2SensorControllerState
{
    Passive,                    // 0
    Standby,                    // 1
    Active,                     // 2
    GNCOnly,                    // 3
    InternalOnly,               // 4
    ControllerState_SIZE,       // 5
};

#endif