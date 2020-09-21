/*
 * uart.h
 *
 * Created: 8/10/2020 2:51:27 PM
 *  Author: BharathVBhatt
 */ 


#ifndef UART_H_
#define UART_H_

#include "avr/io.h"
#include "stdio.h"

extern FILE uartout;

void USART_Init();
void println(uint8_t message[]);
unsigned char USART_Receive(FILE *stream);
void USART_Transmit(char data, FILE  *stream);
#endif /* UART_H_ */