#ifndef TANKPRESSCONTROLLERDEFINITIONSPASABANG_H
#define TANKPRESSCONTROLLERDEFINITIONSPASABANG_H

#include "TankPressControllerClass.h"
#include <array>
#include "ALARApinDefines.h"


#define NUM_TANKPRESSCONTROLLERS 3
//Renegade SF
/* TankPressController HiPressTankController{1, 2, 5000};
TankPressController LoxTankController{2, 3, 900};
TankPressController FuelTankController{3, 3, 900}; */
//Banger SF
TankPressController HiPressTankController{1, 8, 5000, true};
TankPressController LoxTankController{2, 8, 900, true};
TankPressController FuelTankController{3, 8, 900, true};

//
std::array<TankPressController*, NUM_TANKPRESSCONTROLLERS> tankPressControllerArray{&HiPressTankController, &LoxTankController, &FuelTankController};

#endif