/***********************************************************************************************************************************************/
/*
 * File name	: main.cpp
 * Created on	: May 29, 2019
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 * Main function is defined here
 *
 ***********************************************************************************************************************************************/

#ifndef SERIAL_TCP_DATACONFIG_H_
#define SERIAL_TCP_DATACONFIG_H_
/***********************************************************************************************************************************************/
typedef struct sockaddr_in  sockaddr;


#define ipv4address 20 /*Max IPv4 address length*/
typedef struct
{
	long int port;     /*Port*/
	char name[ipv4address];     /*IP*/
}serverlist;

typedef enum {

	//clinet_mypc1,
	clinet_mypc2,
    server_web,
	max_data
} tcp_data_en;






#endif /* SERIAL_TCP_DATACONFIG_H_ */
