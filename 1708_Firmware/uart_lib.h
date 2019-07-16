/*
 * IncFile1.h
 *
 * Created: 6/19/2018 8:25:48 AM
 *  Author: kangm
 */ 


#ifndef UART_LIB_H
#define UART_LIB_H

#include "avr/io.h"

void initUSART(uint8_t baud_rate_code);

void transmit_byte( uint8_t data );

uint8_t receive_byte(void);
 
void print_bytes(uint8_t myBytes[], uint8_t len, int check_sum_enable);

void GetHexString(uint8_t byte, char *out);

uint8_t GetByteFromString(char *in);
 
#endif /* INCFILE1_H_ */