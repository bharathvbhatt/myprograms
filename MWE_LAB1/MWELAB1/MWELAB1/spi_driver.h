/*
 * spi_driver.h
 *
 * Created: 8/5/2020 5:34:25 PM
 *  Author: BharathVBhatt
 */ 
#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_
/*****************************************************************INCLUDES*******************************************************************************/
/*##Section to include all header files##*/
#include <avr/io.h>
/*##END##*/
/********************************************************************************************************************************************************/
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
void spi_init();
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
uint8_t spi_readwrite(uint8_t data);
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
void spi_read(uint8_t* data);
/********************************************************************************************************************************************************/
#endif /* SPI_DRIVER_H_ */