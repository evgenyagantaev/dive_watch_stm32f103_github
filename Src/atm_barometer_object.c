#include "main.h"
#include "atm_barometer_object.h"
#include "atm_barometer_interface.h"
#include "rtc_ds3231_interface.h"
#include "pressure_sensor_object.h"


void atm_barometer_action()
{
	// check if hour_old != hour
	uint8_t hours, minutes, seconds;
	rtc_ds3231_get_time(&hours, &minutes, &seconds);
	if(hours_old != hours)
	{
		hours_old = hours;

		// shift pressure log array
		int i;
		for(i=(LOG_LENGTH-1); i>0; i--)
		{
			atm_pressure_log[i] = atm_pressure_log[i-1];
		}
		double P = pressure_sensor_get_pressure();
		atm_pressure_log[0] = (uint32_t)P;
	}
	
}


void atm_barometer_get_history(uint32_t *buffer)
{
	buffer[0] = atm_pressure_log[11];
	buffer[1] = atm_pressure_log[23];
	buffer[2] = atm_pressure_log[35];
	buffer[3] = atm_pressure_log[47];
}
