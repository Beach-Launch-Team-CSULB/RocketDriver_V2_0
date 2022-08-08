#include "ToMillisTimeTracker.h"

time_t inProgramTime;
//time_t updateProgramTime;

int32_t inProgramSECS;
int32_t updateProgramSECS;

elapsedMicros timerSubSecondsMicros;
uint8_t calibrationLoopIterator = 0;

void myTimeTrackingFunction(uint32_t& rocketDriverSeconds, uint32_t& rocketDriverMicros)
{
    //updateProgramSECS = second();
    if (calibrationLoopIterator < 10)       //each time the seconds roll over this adjusts the ellapsedMicros timer to sync up to it
    {
        if (second() != inProgramSECS)
        {
            timerSubSecondsMicros = 0;
            inProgramSECS = second();
            calibrationLoopIterator++;
        }
    }
    inProgramSECS = second();
    timerSubSecondsMicros = (timerSubSecondsMicros % 1000000);
    
    rocketDriverSeconds = second();
    rocketDriverMicros = timerSubSecondsMicros;
}