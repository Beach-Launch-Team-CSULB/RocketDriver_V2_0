#ifndef STATELIST_H
#define STATELIST_H

// This header defines all the tracked states the system an be in, using an enum class for protection

enum class VehicleState
{
    // These are the God States, they can be reached from any position
    setup,      // 0
    passive,    // 1   // All physical outputs disabled
    standby,    // 2
    test,       // 3
    abort,      // 4
    vent,       // 5
    offNominal, // 6   off nominal is for when individual valves are actuated out of sequence


    // These states can only be accessed in sequence, from passive
    HiPressArm,             // 7
    HiPressPressurized,     // 8
    TankPressArm,           // 9
    TankPressPressurized,   // 10
    fireArmed,              // 11
    fire,                   // 12

};

// very preliminary, update as needed while developing Mission State machine
enum class MissionState
{
    // These are the God States, they can be reached from any position
    passive,                // 0
    standby,                // 1
    staticTestArmed,        // 2
    staticTestActive,       // 3
    postTest,               // 4
    prelaunch,              // 5
    ascentRail,             // 6
    ascentFreeThrust,       // 7
    ascentFreeCoast,        // 8
    descentFree,            // 9
    descentPilot,           // 10
    descentDrogue,          // 11
    descentMain,            // 12
    landed,                 // 13
};
#endif