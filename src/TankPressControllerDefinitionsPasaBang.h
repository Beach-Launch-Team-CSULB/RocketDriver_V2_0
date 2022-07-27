#ifndef TANKPRESSCONTROLLERDEFINITIONSPASABANG_H
#define TANKPRESSCONTROLLERDEFINITIONSPASABANG_H

#include "TankPressControllerClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "ValveDefinitionsPasaBang.h"

#define NUM_TANKPRESSCONTROLLERS 3
//Renegade SF
/* TankPressController HiPressTankController{1, 2, 5000};
TankPressController LoxTankController{2, 3, 900};
TankPressController FuelTankController{3, 3, 900}; */
//Banger SF

// hacky bullshit fake valves to pass to controllers
Valve valve1_1{NormalClosed};
Valve valve1_2{NormalClosed};
Valve valve1_3{NormalClosed};
Valve valve2_1{NormalClosed};
Valve valve2_2{NormalClosed};
Valve valve2_3{NormalOpen};
Valve valve3_1{NormalClosed};
Valve valve3_2{NormalClosed};
Valve valve3_3{NormalClosed};

//Valve* loxVentPointer;

TankPressController HiPressTankController{1, 8, &valve1_1, &HiPressVent, &valve1_3, 5000, 6000};
TankPressController LoxTankController{2, 8, &LoxBang, &valve2_2, &LoxVent, 500, 800, 2.5, 1, .2, 1};
TankPressController FuelTankController{3, 8, &FuelBang, &valve3_2, &FuelVent, 500, 800, 2.5, 1, .2, 1};

/* TankPressController HiPressTankController{1, 8, valve1_1, valve1_2, valve1_3, 5000, 6000};
TankPressController LoxTankController{2, 8, valve2_1, valve2_2, valve2_3, 500, 800, 2.5, 1, .2, 1};
TankPressController FuelTankController{3, 8, valve3_1, valve3_2, valve3_3, 500, 800, 2.5, 1, .2, 1};
 */
//
std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS> tankPressControllerArray{&HiPressTankController, &LoxTankController, &FuelTankController};

#endif