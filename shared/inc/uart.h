#ifndef UART_H
#define UART_H

#include "common_defines.h"

void uart_read(uint8_t* data, uint32_t length);
uint8_t uart_read_byte();
void uart_write(uint8_t* data, uint32_t length);
void uart_write_byte(uint8_t data);
void uart_setup();


#endif