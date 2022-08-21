#ifndef RGBLEDCOLORDEFINITIONS_H
#define RGBLEDCOLORDEFINITIONS_H

#include "ALARABoardControllerClass.h"

// Definitions for color presets of 12bit PWM driven RGB LEDs wired for values to be inverted (0 = full on, 4096 off)

// Whites
RGB_12bitColor LED_WHITE{0,0,0};
RGB_12bitColor LED_WHITE_MIN{4093,4093,4093};
RGB_12bitColor LED_WHITE_8percent{3768,3768,3768};
RGB_12bitColor LED_WHITE_50percent{2048,2048,2048};
// Colors
RGB_12bitColor LED_RED{0,4096,4096};
RGB_12bitColor LED_GREEN{4096,0,4096};
RGB_12bitColor LED_BLUE{4096,4096,0};
RGB_12bitColor LED_TEAL{4096,50,500};
RGB_12bitColor LED_ORANGE{0,3700,4096};
RGB_12bitColor LED_YELLOW{750,2800,4096};
RGB_12bitColor LED_VIOLET{30,4096,3500};
RGB_12bitColor LED_LIME{50,500,4096};


#endif