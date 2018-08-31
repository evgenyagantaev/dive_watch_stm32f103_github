#include "gps_object.h"
#include "gps_interface.h"
#include "string.h"




void add_input_char_into_buffer(uint8_t char)
{
	int current_input_index = strlen(gps_input_buffer);
	gps_input_buffer[current_input_index] = char;
	gps_input_buffer[current_input_index + 1] = 0;
}



void set_end_of_string_received_flag()
{
	end_of_string_received_flag = 1;
}




void gps_action()
{

}










