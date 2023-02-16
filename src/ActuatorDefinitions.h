#ifndef ACTUATORDEFINITIONS_H
#define ACTUATORDEFINITIONS_H

#include "ActuatorClass.h"
#include <array>
#include "ALARApinDefines.h"

#ifdef RENEGADESF
#define NUM_ACTUATORS 2
Actuator Engine1TVC_Y(10, 8, LinearServo);
Actuator Engine1TVC_Z(11, 8, LinearServo);

// ADD Actuators TO THIS VALVE ARRAY IN THE FORM: &ACTUATOR
std::array<Actuator*, NUM_ACTUATORS> actuatorArray{&Engine1TVC_Y, &Engine1TVC_Z};
#endif

#endif