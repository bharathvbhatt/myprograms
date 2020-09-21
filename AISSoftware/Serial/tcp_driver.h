/***********************************************************************************************************************************************/
/*
 * File name	: tcp_init.h
 * Created on	: Dec 27, 2018
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#ifndef SERIAL_TCP_DRIVER_H_
#define SERIAL_TCP_DRIVER_H_
/***********************************************************************************************************************************************/

#include "stdio.h"
#include "string.h"
#include "sys/socket.h"
#include "arpa/inet.h"
#include "basictypes.h"
#include "tcp_dataconfig.h"
#include "usbserial_main.h"
#include "stdlib.h"
#include <unistd.h>
#include <signal.h>
#include <sys/sendfile.h>
#include "errno.h"
#include "fcntl.h"
#include "termios.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <netinet/tcp.h>

/***********************************************************************************************************************************************/
/*
 * API				: TCP Init Function
 * Arguments 		: None
 * Function Usage 	: TCP Init
 * Return Type 		: void
 * Return Value 	: none
 *
 ***********************************************************************************************************************************************/
 tcpstate_en tcpinit(int *sock, sockaddr *addr, char ip[],long int port);
/***********************************************************************************************************************************************/
/*
 * API				: TCP State Machine
 * Arguments 		: None
 * Function Usage 	: TCP State loops
 * Return Type 		: void type
 * Return Value 	: none
 *
 ***********************************************************************************************************************************************/
/*void tcp_statemachine();*/
/***********************************************************************************************************************************************/
/*
 * API				: TCP Create Socket
 * Arguments 		: None
 * Function Usage 	: To create a Socket for application to connect to the network
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_socketcreated(success);tcp_socketnotcreated(Failure);
 *
 ***********************************************************************************************************************************************/
/*tcpstate_en create_socket();*/
 /***********************************************************************************************************************************************/
 /*
  * API				: To establish connection
  * Arguments 		: int sock, sockaddr addr , the details of the sockets created is used to set-up a connection
  * Function Usage 	: TCP Connection Establishment
  * Return Type 		: tcpstate_en
  * Return Value 	: tcp_connected(success);tcp_connectionfailed(Failure)
  *
  ***********************************************************************************************************************************************/
tcpstate_en connect_tcp(int sock, sockaddr addr);
/***********************************************************************************************************************************************/
/*
 * API				: To bind server connection
 * Arguments 		: int sock, sockaddr addr , used to bind connection for the socket given
 * Function Usage 	: TCP bind server
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_bind(success);tcp_bindfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en bind_tcp(int sock, sockaddr addr);
/***********************************************************************************************************************************************/
/*
 * API				: To accept TCP server request
 * Arguments 		: None
 * Function Usage 	: TCP accept client
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_accept(success);tcp_acceptfailed(Failure)
 *
 **********************************************************************************************************************************************/
tcpstate_en accept_tcp(int sock, sockaddr client);
/***********************************************************************************************************************************************/
/*
 * API				: To read the client sent request
 * Arguments 		: char client_message[] , buffer to hold the read message
 * Function Usage 	: TCP read client info/request
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_read(success);tcp_readfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en read_tcp( char client_message[] );
/***********************************************************************************************************************************************/
/*
 * API				: To send data on TCP
 * Arguments 		: int sock, char buf[] , socket to send data on , Buffer of data to be sent
 * Function Usage 	: TCP read
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_send(success);tcp_sendfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en send_tcp(int sock, char buf[] );
/***********************************************************************************************************************************************/
#endif /* SERIAL_TCP_DRIVER_H_ */
