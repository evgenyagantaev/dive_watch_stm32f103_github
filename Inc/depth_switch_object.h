#ifndef DEPTH_SWITCH_OBJECT_H
#define DEPTH_SWITCH_OBJECT_H

#include "main.h"
#include "gpio.h"


#define DEPTH0 1.0
#define DEPTH1 15.0
#define DEPTH2 20.0
#define DEPTH3 25.0
#define DEPTH4 30.0



static double current_depth = DEPTH1;

static int depth_switch_key_press_period_counter = 0;







#endif
