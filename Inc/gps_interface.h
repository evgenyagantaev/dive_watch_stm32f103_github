#ifndef GPS_INTERFACE_H
#define GPS_INTERFACE_H
#include "main.h"



void gps_object_init();
void add_input_char_into_buffer(uint8_t byte);
void set_end_of_string_received_flag();

void gps_action();












#endif
