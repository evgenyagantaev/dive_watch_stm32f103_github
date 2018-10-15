#include "dive_meter_object.h"
#include "dive_meter_interface.h"

				static uint8_t depth;

void dive_meter_action()
{
	double P = pressure_sensor_get_pressure();
	uint32_t surface_pressure = (uint32_t)atm_barometer_get_mean_pressure();
	if(P > surface_pressure)
		current_depth = (uint8_t)((double)(P - surface_pressure))/9800.0;
	else
		current_depth = 0;
				//debug
				depth += 1;
				if(depth > 11)
					depth = 7.0;
				current_depth = depth;
				//debug
	
	uint8_t seconds, minutes, hours;
	rtc_ds3231_get_time(&hours, &minutes, &seconds);

	if(!dive_started_flag)
	{
		dive_started_flag = 1;

		current_dive_time = 0;
		old_minutes = minutes;
		max_depth = current_depth;
	}
	else
	{
		if(old_minutes != minutes)
		{
			old_minutes = minutes;
			current_dive_time++;
		}

		if(current_depth > max_depth)
			max_depth = current_depth;

	}
}



uint8_t dive_meter_get_current_depth()
{
	return current_depth;
}
uint8_t dive_meter_get_current_dive_time()
{
	return current_dive_time;
}
uint8_t dive_meter_get_max_depth()
{
	return max_depth;
}


void dive_meter_drop_dive()
{
	if(dive_started_flag)
		dive_started_flag = 0;
}


