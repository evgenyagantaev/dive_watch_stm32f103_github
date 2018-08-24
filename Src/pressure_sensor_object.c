#include "pressure_sensor_object.h"

//****************************************************************************
uint8_t write_byte(uint8_t data)
{

	uint8_t data_out;
    uint8_t read_data;

	// wait for spi transmitter readiness
	while ((SPI1->SR & SPI_SR_TXE) == RESET );
	data_out = data;
    SPI1->DR = data_out;
    // wait while a transmission complete
	while ((SPI1->SR & SPI_SR_RXNE) == RESET );
    read_data = SPI1->DR;
	
	return read_data;

	
}

//****************************************************************************
void pressure_sensor_object_init()
{
	int i;

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//                 RESET
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// reset spi1 cs pin
    spi1_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi1_cs_pressure_Pin << 16); 	// reset
	// transmit 0x1e                             	
	write_byte( 0x1e);                         	
	// set spi1 cs pin                           	
    spi1_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi1_cs_pressure_Pin ;	// set
	HAL_Delay(3);
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	for(i=1; i<7; i++)
	{
		//send read prom command
		// reset spi1 cs pin
    	spi1_cs_pressure_GPIO_Port->BSRR = (uint32_t)(spi1_cs_pressure_Pin << 16); 	// reset
		// transmit command with address 
		write_byte( 0xa0 + (((uint8_t)i)<<1));

		// read ms byte
		sensor_prom[i] = write_byte(0x55);
		sensor_prom[i] <<= 8;
		// read ls byte
		sensor_prom[i] += write_byte(0x55);

		// set spi1 cs pin
    	spi1_cs_pressure_GPIO_Port->BSRR = (uint32_t)spi1_cs_pressure_Pin ;	// set
	}
}
