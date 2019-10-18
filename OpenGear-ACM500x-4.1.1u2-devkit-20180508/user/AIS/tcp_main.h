/***********************************************************************************************************************************************/
/*
 *	File name 	: tcp_main.h
 *  Created on	: Dec 20, 2018
 *  Author    	: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
/*Add function declarations and header files needed for the programs*/
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>     
#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/sendfile.h>
#include "errno.h"
#include "fcntl.h"
#include "termios.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <netinet/tcp.h>
#include "datastruct.h"

/***********************************************************************************************************************************************/
/*
 * API				: TCP int Function
 * Arguments 		: none
 * Function Usage 	: TCP socket creation and initialization
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void init_tcp();
/***********************************************************************************************************************************************/
/*
 * API				: TCP Clientfault handling
 * Arguments 		: void
 * Function Usage 	: To handle Connect lost and other small socket faults(Detailed error handling is not done)
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void tcp_Clientfaulthandling();

/***********************************************************************************************************************************************/
/*
 * API				: TCP Serverfault handling
 * Arguments 		: void
 * Function Usage 	: To handle Connect lost and other small socket faults(Detailed error handling is not done)
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void tcp_Serverfaulthandling();
/***********************************************************************************************************************************************/
/*
 * API				: TCP main function
 * Arguments 		: char tcpbuf[], contains the AIS data to be sent to the server (Client Function)
 * Function Usage 	: To send the TCP data to server.
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void tcp_main(char tcpbuf[]);
/***********************************************************************************************************************************************/
/*
 * API				: TCP server initialization
 * Arguments 		: void
 * Function Usage 	: Init of server, bind and listen functionality is taken care.
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void server_init();
/***********************************************************************************************************************************************/
/*
 * API				: Thread to handle the server implementation
 * Arguments 		: void
 * Function Usage 	: Thread to handle the server implementation
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void thr(void);
/***********************************************************************************************************************************************/
/*
 * API				: Server program to check IRQ
 * Arguments 		: void
 * Function Usage 	: TO check any client has requested for service
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void check_irq();
/***********************************************************************************************************************************************/
/*
 * API				: Main HTTP server implementation
 * Arguments 		: char buff[], consists the stream/string of HTTP request
 * Function Usage 	: TO check any client has requested for service
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void server_send( char buff[]);
/***********************************************************************************************************************************************/

