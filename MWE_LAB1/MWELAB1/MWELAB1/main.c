/*
 * MWELAB1.c
 *
 * Created: 8/5/2020 2:02:14 PM
 * Author : BharathVBhatt
 */ 
#define F_CPU 16000000

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include "uart.h"
#include "spi_driver.h"

extern FILE uartout;

int main(void)
{
    /* Replace with your application code */
	uint8_t data=0;
	spi_init();
    USART_Init();
	
    while (1) 
    {
		
		data=spi_readwrite(12);
		printf("data is :%x\n",data);
		_delay_ms(200);
   }
	return 0;
}

