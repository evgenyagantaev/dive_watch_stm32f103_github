#include "one_second_timer_object.h"


void one_second_timer_init();
void one_second_timer_start()
{
	HAL_TIM_Base_Start_IT(&htim2);
}
void one_second_timer_stop()
{
	HAL_TIM_Base_Stop_IT(&htim2);
}
