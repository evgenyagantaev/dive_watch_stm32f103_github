#ifndef AT24C32_INTERFACE_H
#define AT24C32_INTERFACE_H

void at24c32_write_32(uint16_t address, uint32_t data);
void at24c32_read_32(uint16_t address, uint32_t *data);



#endif
