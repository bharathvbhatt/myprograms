/***********************************************************************************************************************************************/
/*
 * File name	: usb_init.c
 * Created on	: Dec 27, 2018
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#include "usb_init.h"
#include "usbserial_main.h"
#include <sys/time.h>
#include "time.h"
#include <signal.h>
#include <unistd.h>

int comport=0;
struct termios SerialPortSettings;	/* Create the structure                          */
/***************************************************************************************************************************************/
/*
 * API				: USB Init Function
 * Arguments 		: None
 * Function Usage 	: USB Initialization
 * Return Type 		: void
 * Return Value 	: none
 *
 ***************************************************************************************************************************************/
void usbinit()
{
	/*Init function sets the USB state machine state*/
	printf("Serial Communication on Raspberry Pi:\n");
	usbstatus.usbsts=usb_init;
	/*Closes any unwanted open port*/
	close(comport);
	/*Just wait for system init to complete*/
	/*sleep(40);*/
}

/***************************************************************************************************************************************/
/*
 * API				: USB State Machine
 * Arguments 		: None
 * Function Usage 	: USB State loops
 * Return Type 		: void
 * Return Value 	: none
 *
 *
 ***************************************************************************************************************************************/
void usb_statemachine()
{
	/*Main state machine for USB control*/
	switch(usbstatus.usbsts)
	{
	case usb_init 			: usbstatus.usbsts = usb_openport();
	                          break;
	case usb_openfailed 	: usbstatus.usbsts=usb_init;
	                          break;
	case usb_open 			: usbstatus.usbsts = usb_setat();
	                          break;
	case usb_atsetfailed 	: usbstatus.usbsts = usb_open;
	                          break;
	case usb_atset      	: usbstatus.usbsts =usb_read;
                              break;
	case usb_read			: usb_main(comport);
	                          break;
    case usb_readfailed 	: usbstatus.usbsts=usb_init;
	                          close(comport);
	                          while(usb_openport()!=usb_open){;}
	                          close(comport);
                              break;
    default 				: break;
	}
}

/***************************************************************************************************************************************/
/*
 * API				: USB Port Open
 * Arguments 		: None
 * Function Usage 	: To open the USB port
 * Return Type 		: usbstate_en
 * Return Value 	: usb_open(success); usb_atsetfailed (Failed) ;
 *
 ***************************************************************************************************************************************/

usbstate_en usb_openport()
{

	usbstate_en Return=usb_open;


	/*Open the USB Port*/
	comport =open(OpenPort, O_RDWR | O_NOCTTY | O_NDELAY);
	if(comport < 0)
	{
		/*Port not opened*/
		Return = usb_openfailed;

	}

	else
	{
		printf("COMport Opened\n");
	}

	return (Return);
}
/***************************************************************************************************************************************/
/*
 * API				: USB Set Attribute
 * Arguments 		: None
 * Function Usage 	: To set USB read baud rate and read mode
 * Return Type 		: usbstate_en
 * Return Value 	: usb_atset(success); usb_openfailed (Failed) ;
 *
 ***************************************************************************************************************************************/
usbstate_en usb_setat()
{

	usbstate_en Return =usb_atset;

	fcntl(comport, F_SETFL, 0);
	tcgetattr(comport, &SerialPortSettings);									/* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings,B38400); 									/* Set Read  Speed as 38400*/

	/* To set below flags and to get data reception working,
	 * initially command  : stty raw -echo ospeed 38400 < /dev/ttyUSB* is used
	 * values of the below flags are noted and these flags are configured accordingly.*/

	SerialPortSettings.c_cflag &= ~PARENB;    									/*Disables the Parity Enable bit(PARENB),So No Parity*/
	SerialPortSettings.c_cflag &= ~CSTOPB;    									/*CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit*/
	SerialPortSettings.c_cflag &= ~CSIZE;	 									/*Clears the mask for setting the data size*/
	SerialPortSettings.c_cflag |= CS8;        									/*Set the data bits = 8*/
	SerialPortSettings.c_cflag &= ~CRTSCTS;       								/* No Hardware flow Control */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; 								/* Enable receiver,Ignore Modem Control lines*/



	SerialPortSettings.c_iflag =0;
	SerialPortSettings.c_lflag = 35376;
	SerialPortSettings.c_oflag &= ~OPOST; 										/*No Output Processing*/

	/* Setting Time outs*/
	SerialPortSettings.c_cc[VMIN] = 255;  										/*Read at least 10 characters*/
	SerialPortSettings.c_cc[VTIME] = 0;   										/*Wait indefinetly*/
	if((tcsetattr(comport,TCSANOW,&SerialPortSettings)) != 0) 			 		/* Set the attributes to the termios structure */
	{
		printf("\n  ERROR ! in Setting attributes");
		Return =usb_atsetfailed;


	}
	else
	{
		printf("\n  BaudRate = B38400 \n  StopBits = 1 \n  Parity   = none\n");
	}

	return (Return);
}

/**************************************************************************************************************************************/
















