#ifndef ONE_SECOND_TIMER_OBJ_H
#define ONE_SECOND_TIMER_OBJ_H

#include "tim.h"

static int one_second_timer_flag;



void one_second_timer_set_flag();
void one_second_timer_reset_flag();
int one_second_timer_get_flag();

void one_second_timer_init();
void one_second_timer_start();
void one_second_timer_stop();

#endif
