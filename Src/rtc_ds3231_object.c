#include "rtc_ds3231_object.h"
#include "rtc_ds3231_interface.h"



void rtc_ds3231_get_date(uint32_t *year, uint8_t *month, uint8_t *date);
void rtc_ds3231_get_time(uint8_t *Hours, uint8_t *Minutes, uint8_t *Seconds)
{
	*Hours = hours;
	*Minutes = minutes;
	*Seconds = seconds;
}

void rtc_ds3231_set_date(uint32_t year, uint8_t month, uint8_t date);
void rtc_ds3231_set_time(uint8_t Hours, uint8_t Minutes, uint8_t Seconds)
{
	ds3231_buffer[0] = 0;
	ds3231_buffer[1] = 0;
	ds3231_buffer[2] = 0;

	HAL_I2C_Master_Transmit(ds3231_i2c_handle, ds3231_shifted_address, ds3231_buffer, DS3231_BUFFER_LENGTH, 500);
}

void rtc_ds3231_set_i2c_handle(I2C_HandleTypeDef *handle)
{
	ds3231_i2c_handle = handle;
}


void rtc_ds3231_action()
{

	HAL_I2C_Master_Receive(ds3231_i2c_handle, ds3231_shifted_address, ds3231_buffer, DS3231_BUFFER_LENGTH, 500);

	seconds = 10*(ds3231_buffer[0]>>4) + (ds3231_buffer[0] & 0x0f);
	minutes = 10*(ds3231_buffer[1]>>4) + (ds3231_buffer[1] & 0x0f);
	hours = 10*((ds3231_buffer[2]>>4) & 0x03) + (ds3231_buffer[2] & 0x0f);
}









