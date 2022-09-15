#ifndef TANKPRESSCONTROLLERDEFINITIONS_H
#define TANKPRESSCONTROLLERDEFINITIONS_H

#include "TankPressControllerClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "ValveDefinitions.h"

#ifdef PASABANG
#define NUM_TANKPRESSCONTROLLERS 3
//Banger SF

// hacky bullshit fake valves to pass to controllers as defaults
Valve valve1_1{NormalClosed};
Valve valve1_2{NormalClosed};
Valve valve1_3{NormalClosed};
Valve valve2_1{NormalClosed};
Valve valve2_2{NormalClosed};
Valve valve2_3{NormalOpen};
Valve valve3_1{NormalClosed};
Valve valve3_2{NormalClosed};
Valve valve3_3{NormalClosed};
//
TankPressController HiPressTankController{2, 8, &valve1_1, &HiPressVent, &valve1_3, 6000};
TankPressController LoxTankController{3, 8, &LoxBang, &valve2_2, &LoxVent, 300, 60, 600, 2.5, 1.0, 1.5, 1};
TankPressController FuelTankController{4, 8, &FuelBang, &valve3_2, &FuelVent, 300, 60, 600, 2.5, 1.0, 1.5, 1};
//
std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS> tankPressControllerArray{&HiPressTankController, &LoxTankController, &FuelTankController};
#endif

#ifdef RENEGADESF
#define NUM_TANKPRESSCONTROLLERS 3
// hacky bullshit fake valves to pass to controllers as defaults
//Valve valve1_1{NormalClosed};
//Valve valve1_2{NormalClosed};
Valve valve1_3{NormalClosed};
//Valve valve2_1{NormalClosed};
//Valve valve2_2{NormalClosed};
//Valve valve2_3{NormalOpen};
//Valve valve3_1{NormalClosed};
//Valve valve3_2{NormalClosed};
//Valve valve3_3{NormalClosed};
//
TankPressController HiPressTankController{2, 2, &HiPress, &HiPressVent, &valve1_3, 6000};
TankPressController LoxTankController{3, 3, &LoxDomeReg, &LoxDomeRegVent, &LoxVent, 300, 60, 725, 2.5, 1.0, 1.5, 1, true, false};
TankPressController FuelTankController{4, 3, &FuelDomeReg, &FuelDomeRegVent, &FuelVent, 300, 60, 725, 2.5, 1.0, 1.5, 1, true, false};
//
std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS> tankPressControllerArray{&HiPressTankController, &LoxTankController, &FuelTankController};


#endif

#endif