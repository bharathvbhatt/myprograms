/***********************************************************************************************************************************************/
/*
 * File name	: usbserial.c
 * Created on	: Dec 20, 2018
 * Author		: Bharath Vikram Bhatt <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#ifndef SERIAL_USBSERIAL_MAIN_H_
#define SERIAL_USBSERIAL_MAIN_H_
/***********************************************************************************************************************************************/

/*Add function declarations and header files needed for the programs*/
/***********************************************************************************************************************************************/
/*
 * API				: USB Main Function
 * Arguments 		: None
 * Function Usage 	: USB read
 * Return Type 		: Integer type
 * Return Value 	: 0 on successful execution
 *
 ***********************************************************************************************************************************************/
int usb_main(int file_st);
/***********************************************************************************************************************************************/
/*
 * API				: Clean Buffer
 * Arguments 		: None
 * Function Usage 	: Clean all buffers once the buffered data is sent over TCP
 * Return Type 		: boolean
 * Return Value 	: True
 *
 ***********************************************************************************************************************************************/
boolean clean_buf();
/************************************************************************************************************************************************
 *
 * API				: Get unix time-stamp
 * Arguments 		: char time[] : the refernce passing buffer with timer value
 * Function Usage 	: To get time-stamp
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void get_timerval(char time[]);

#endif /* SERIAL_USBSERIAL_MAIN_H_ */
