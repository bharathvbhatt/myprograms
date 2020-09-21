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
#include "rfic_Init.h"


int main(void)
{
    /* Replace with your application code */
	uint8_t data=0;
	spi_init();
    USART_Init();
	DDRB |=(1<<1);
	PORTB &= ~(1<<1);
    serialModeRx_init();
	serialModeWrite();
    while (1) 
    {
		 serialModeWrite();
		 _delay_ms(1000);
    }
	return 0;
}

