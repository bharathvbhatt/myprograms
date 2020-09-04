/*
 * uart.c
 *
 * Created: 8/10/2020 2:51:11 PM
 *  Author: BharathVBhatt
 */ 

#include "avr/io.h"
#include "avr/interrupt.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "uart.h"

#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

FILE uartout = FDEV_SETUP_STREAM(USART_Transmit, USART_Receive, _FDEV_SETUP_RW);


void USART_Init()
{
	/*Set baud rate */
	UBRR0H = ((MYUBRR)>>8);
	UBRR0L =  MYUBRR;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	stdin=stdout=&uartout;
}


unsigned char USART_Receive(FILE *stream)
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}

void USART_Transmit(char data, FILE  *stream)
{
	/* Wait for empty transmit buffer */
   if(data == '\n')
   {
	   USART_Transmit('\r', 0);
   }
	while (!(UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
	
}