#ifndef DIVE_METER_INTERFACE_H
#define DIVE_METER_INTERFACE_H

#include "dive_meter_object.h"

void dive_meter_action();

uint8_t dive_meter_get_current_depth();
uint8_t dive_meter_get_current_dive_time();
uint8_t dive_meter_get_max_depth();

void dive_meter_drop_dive();


#endif
