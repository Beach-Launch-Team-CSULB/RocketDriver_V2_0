#ifndef STATELIST_H
#define STATELIST_H

// This header defines all the tracked states the system an be in, using an enum class for protection

enum class VehicleState
{
    // These are the God States, they can be reached from any position
    setup,      // 0
    debug,      // 1   the outputs disabled for testing state?
    passive,    // 2
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
    staticTestArmed,        // 1
    staticTestActive,       // 2
    postTest,               // 3
    prelaunch,              // 4
    ascentRail,             // 5
    ascentFreeThrust,       // 6
    ascentFreeCoast,        // 7
    descentFree,            // 8
    descentPilot,           // 9
    descentDrogue,          // 10
    descentMain,            // 11
    landed,                 // 12
};
#endif