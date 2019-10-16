/***********************************************************************************************************************************************/
/*
 * usb_init.h
 *
 *  Created on: Dec 27, 2018
 *      Author: bharath
 *
 ***********************************************************************************************************************************************/
#ifndef SERIAL_USB_INIT_H_
#define SERIAL_USB_INIT_H_
/***********************************************************************************************************************************************/
#include "errno.h"
#include "fcntl.h"
#include "termios.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <sys/types.h>
#include <unistd.h>
#include "basictypes.h"
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include "tcp_main.h"

/***********************************************************************************************************************************************/
#define OpenPort "/dev/ttyUSB0" /*Listen to USB */

/***************************************************************************************************************************************/
/*
 * API				: USB Init Function
 * Arguments 		: None
 * Function Usage 	: USB Initialization
 * Return Type 		: void
 * Return Value 	: none
 *
 ***************************************************************************************************************************************/
void usbinit();

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
void usb_statemachine();
/***************************************************************************************************************************************/
/*
 * API				: USB Port Open
 * Arguments 		: None
 * Function Usage 	: To open the USB port
 * Return Type 		: usbstate_en
 * Return Value 	: usb_open(success); usb_atsetfailed (Failed) ;
 *
 ***************************************************************************************************************************************/
usbstate_en usb_openport();
/***************************************************************************************************************************************/
/*
 * API				: USB Set Attribute
 * Arguments 		: None
 * Function Usage 	: To set USB read baud rate and read mode
 * Return Type 		: usbstate_en
 * Return Value 	: usb_atset(success); usb_openfailed (Failed) ;
 *
 ***************************************************************************************************************************************/
usbstate_en usb_setat();

#endif /* SERIAL_USB_INIT_H_ */
