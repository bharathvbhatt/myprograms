/*
 * spi_driver.c
 *
 * Created: 8/5/2020 5:34:01 PM
 *  Author: BharathVBhatt
 */ 
/*****************************************************************INCLUDES*******************************************************************************/
/*##Section to include all header files##*/

#include <avr/io.h>
#include <stdio.h>
#include <stdbool.h> 
#include <avr/interrupt.h>
/*##END##*/
/*******************************************************************DEFINES*******************************************************************************/
#define SPI_DDR DDRB
#define CS      PINB2
#define MOSI    PINB3
#define MISO    PINB4
#define SCK     PINB5


/*****************************************************************FUNCTIONS******************************************************************************/
/*##Section for functions##*/

/** Function Name (Public)   : spi_init
 *  
 *  Function Description     : To initialize the spi communication
 *  
 *  Arguments                : void
 *  
 *  Return                   : void
 *  
 *  Usage guide & scheduling :
 *        Here registers for SPI functionality for the uC are set
 *        Called by functions using SPI
 *  
 *  Remarks :
 *        Always be initialized before use!!
 * ******************************************************************************************************************************************************/
void spi_init()
{
	 /* set CS, MOSI and SCK to output*/
	SPI_DDR |= (1 << CS) | (1 << MOSI) | (1 << SCK);
	/*
	SPIE : 0 -> SPI interrupt disabled ; 1-> SPI interrupt enabled
	SPE  : Must be one for all SPI operations
	MSTR : Setting this bit makes the device as master 
	CPHA : Setting this bit makes SCK zero when idle. 
	SPI2x SPR1 SPR0 is 000 so the effective clock frequnecy is flck/4 ->250ns
	*/
   SPCR |= (1<<SPIE)|(1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1) ; //|(1<<CPHA);
  // SPSR |=(1<<SPI2X);
   /*Master mode SS(slave select) is made as output*/
   DDRB|=(1<<PINB2);
   DDRB |=(1<<1);
   PORTB &=~(1<<1);
}
/********************************************************************************************************************************************************/
/** Function Name (Public)   : spi_write
 *  
 *  Function Description     : To write the spi data
 *  
 *  Arguments                : 
 * 
 *            uint8_t data : 8 bit data to be transmitted 
 *   
 *  Return                   : void
 *  
 *  Usage guide & scheduling :
 *        Here registers SPDR, SPI data register is written to transmit data
 *        Called by functions to transmit data
 *  
 *  Remarks :
 *        
 * ******************************************************************************************************************************************************/
uint8_t spi_readwrite(uint8_t data)
{

   SPDR = data;
   while(!(SPSR & (1 << SPIF)));
   return SPDR;
}
/********************************************************************************************************************************************************/
/** Function Name (Public)   : spi_read
 *  
 *  Function Description     : To read the spi data
 *  
 *  Arguments                : 
 * 
 *            uint8_t *data : 8 bit data received
 *   
 *  Return                   : void
 *  
 *  Usage guide & scheduling :
 *        Here registers SPDR, SPI data register is read for data
 *        Called by functions to receive data
 *  
 *  Remarks :
 *        
 * ******************************************************************************************************************************************************/
void spi_read(uint8_t* data)
{
	*data = SPDR;
}
/********************************************************************************************************************************************************/
/** Function Name (Public)   : ISR
 *  
 *  Function Description     : Interrupt service routine for SPI data transfer complete
 *  
 *  Arguments                : void
 *   
 *  Return                   : void
 *  
 *  Usage guide & scheduling :
 *        Interrupt is raised when SPI transmit is successful.
 *        This sets the bit in SPSR register
 *  
 *  Remarks :
 *        
 * ******************************************************************************************************************************************************/
ISR(SPI_STC_vect)
{
    
}
/********************************************************************************************************************************************************/