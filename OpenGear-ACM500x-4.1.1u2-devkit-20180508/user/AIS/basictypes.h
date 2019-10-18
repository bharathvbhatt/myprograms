 /*
 * basictypes.h
 *
 *  Created on: Jan 3, 2019
 *      Author: bharath
 */

#ifndef SERIAL_BASICTYPES_H_
#define SERIAL_BASICTYPES_H_

#include "stdio.h"
/********************************************Compiler Switches*******************************************************************************/
#define Client_software_cs
#define Server_software_cs


/********************************************************************************************************************************************/

typedef enum{
	False,
	True
}boolean;
/*State Machine*/
typedef enum {
	state_init,
	state_main,
	state_err
}statetranstion_en;

extern statetranstion_en Currentst;

typedef enum{
	usb_init,
	usb_open,
	usb_openfailed,
	usb_atset,
	usb_atsetfailed,
	usb_read,
	usb_readfailed,
	err_handling
}usbstate_en;

typedef struct {

	usbstate_en usbsts;

}usbstatus_st;

extern usbstatus_st  usbstatus ;


typedef enum{
	tcp_init,
	tcp_socketcreated,
	tcp_socketnotcreated,
	tcp_connected,
	tcp_connectionfailed,
	tcp_bind,
	tcp_bindfailed,
	tcp_listen,
	tcp_accept,
	tcp_acceptfailed,
	tcp_read,
	tcp_readfailed,
	tcp_send,
	tcp_sendfailed
}tcpstate_en;

typedef struct {

	tcpstate_en tcpsts;

}tcpstatus_st;





#endif /* SERIAL_BASICTYPES_H_ */
