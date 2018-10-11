#include "main.h"
#include "at24c32_object.h"
#include "at24c32_interface.h"


void at24c32_set_i2c_handle(I2C_HandleTypeDef *handle)
{
	at24c32_i2c_handle = handle;
}

void at24c32_write_32(uint16_t address, uint32_t data)
{
	//
	uint8_t b0, b1, b2, b3;
	b0 = (uint8_t)(data & 0x000000ff);
	b1 = (uint8_t)((data>>8) & 0x000000ff);
	b2 = (uint8_t)((data>>16) & 0x000000ff);
	b3 = (uint8_t)(data>>24);

	HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, address, 2, &b0, 1, 100);
	HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, address+1, 2, &b1, 1, 100);
	HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, address+2, 2, &b2, 1, 100);
	HAL_I2C_Mem_Write(at24c32_i2c_handle, at24c32_shifted_address, address+3, 2, &b3, 1, 100);
	

}


void at24c32_read_32(uint16_t address, uint32_t *data)
{
	uint8_t b0, b1, b2, b3;

	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, address, 2, &b0, 1, 100);
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, address+1, 2, &b1, 1, 100);
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, address+2, 2, &b2, 1, 100);
	HAL_I2C_Mem_Read(at24c32_i2c_handle, at24c32_shifted_address, address+3, 2, &b3, 1, 100);

	uint32_t return_value = b0 + (((uint32_t)b1)<<8) + (((uint32_t)b2)<<16) + (((uint32_t)b3)<<24);

	*data = return_value;

}
