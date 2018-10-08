#include "gps_object.h"
#include "gps_interface.h"
#include "string.h"



void gps_object_init()
{
	gps_input_buffer[0] = 0;
}

void add_input_char_into_buffer(uint8_t byte)
{
	int current_input_index = strlen(gps_input_buffer);
	gps_input_buffer[current_input_index] = byte;
	gps_input_buffer[current_input_index + 1] = 0;


	if((current_input_index+1) >= GPS_INPUT_BUFFER_LENGTH) // buffer overflow
		gps_input_buffer[0] = 0; // reset buffer

}



void set_end_of_string_received_flag()
{
	end_of_string_received_flag = 1;
}




void gps_action()
{

}










