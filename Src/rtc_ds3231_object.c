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
	ds3231_buffer[0] = ((uint8_t)(Seconds/10))<<4 + ((uint8_t)(Seconds%10));
	ds3231_buffer[1] = ((uint8_t)(Minutes/10))<<4 + ((uint8_t)(Minutes%10));
	ds3231_buffer[2] = ((uint8_t)(Hours/10))<<4 + ((uint8_t)(Hours%10));

	uint16_t data_array_length = 1;

	//HAL_I2C_Master_Transmit(ds3231_i2c_handle, ds3231_shifted_address, ds3231_buffer, DS3231_BUFFER_LENGTH, 500);
	HAL_I2C_Mem_Write(ds3231_i2c_handle, ds3231_shifted_address, 0x0, 1, &(ds3231_buffer[0]), data_array_length, 500);
	HAL_I2C_Mem_Write(ds3231_i2c_handle, ds3231_shifted_address, 0x1, 1, &(ds3231_buffer[1]), data_array_length, 500);
	HAL_I2C_Mem_Write(ds3231_i2c_handle, ds3231_shifted_address, 0x2, 1, &(ds3231_buffer[2]), data_array_length, 500);
}

void rtc_ds3231_set_i2c_handle(I2C_HandleTypeDef *handle)
{
	ds3231_i2c_handle = handle;
}


void rtc_ds3231_action()
{

	uint16_t data_array_length = 3;

	//HAL_I2C_Master_Receive(ds3231_i2c_handle, ds3231_shifted_address, ds3231_buffer, DS3231_BUFFER_LENGTH, 500);
	HAL_I2C_Mem_Read(ds3231_i2c_handle, ds3231_shifted_address, 0x0, 1, ds3231_buffer, data_array_length, 500);

	seconds = 10*(ds3231_buffer[0]>>4) + (ds3231_buffer[0] & 0x0f);
	minutes = 10*(ds3231_buffer[1]>>4) + (ds3231_buffer[1] & 0x0f);
	hours = 10*((ds3231_buffer[2]>>4) & 0x03) + (ds3231_buffer[2] & 0x0f);
}









