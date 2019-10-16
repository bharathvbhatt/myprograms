/***********************************************************************************************************************************************/
/*
 * File name	: tcp_init.c
 * Created on	: Dec 27, 2018
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#include "tcp_driver.h"
#include "tcp_main.h"
/***********************************************************************************************************************************************/
/*
 * API				: TCP Init Function
 * Arguments 		: int *sock, sockaddr *addr, char ip[],long int port , used to set up a socket
 * Function Usage 	: To start the TCP connection by opening socket
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_socketcreated (success);tcp_socketnotcreated(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en tcpinit(int *sock, sockaddr *addr, char ip[],long int port)
{
	tcpstate_en Return_st=tcp_socketcreated;
	printf("Initialization:\n");


		addr->sin_addr.s_addr = inet_addr(ip);
		addr->sin_family = AF_INET;
		addr->sin_port = htons(port);

		*sock = socket(AF_INET , SOCK_STREAM , 0);               /*Create Socket*/
			if(*sock== -1)
			{
				printf("Could not create socket \n");
				Return_st=tcp_socketnotcreated;
			}
			else
			{
				printf("Socket created\n");
			}

	return (Return_st);

}
/***********************************************************************************************************************************************/
/*
 * API				: To establish connection
 * Arguments 		: int sock, sockaddr addr , the details of the sockets created is used to set-up a connection
 * Function Usage 	: TCP Connection Establishment
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_connected(success);tcp_connectionfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en connect_tcp(int sock, sockaddr addr)
{

	tcpstate_en Return_st=tcp_connected;
	if (connect(sock , (struct sockaddr *)&addr , sizeof(addr)) < 0)
	{

		printf("connect failed. Error\n");
	    Return_st=tcp_connectionfailed;


	}
	else
	{
		printf("Connected\n");
	}
	return (Return_st);
}

/***********************************************************************************************************************************************/
/*
 * API				: To bind server connection
 * Arguments 		: int sock, sockaddr addr , used to bind connection for the socket given
 * Function Usage 	: TCP bind server
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_bind(success);tcp_bindfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en bind_tcp(int sock, sockaddr addr)
{
	tcpstate_en Return_st=tcp_bind;
	if( bind(sock,(struct sockaddr *)&addr , sizeof(addr)) < 0)
	{
			/*print the error message*/
			perror("bind failed. Error");
		    Return_st = tcp_bindfailed;
	}
	else
	{
		printf("bind done\n");
	}
return (Return_st);
}
/***********************************************************************************************************************************************/
/*
 * API				: To accept TCP server request
 * Arguments 		: None
 * Function Usage 	: TCP accept client
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_accept(success);tcp_acceptfailed(Failure)
 *
 **********************************************************************************************************************************************/
tcpstate_en accept_tcp(int sock, sockaddr client)
{
	tcpstate_en Return_st=tcp_accept;
	client_sock = accept(sock, (struct sockaddr *)&client, (socklen_t*)&c);
			if (client_sock<0)
			{
         		Return_st=tcp_acceptfailed;
			}
	return (Return_st);
}
/***********************************************************************************************************************************************/
/*
 * API				: To read the client sent request
 * Arguments 		: char client_message[] , buffer to hold the read message
 * Function Usage 	: TCP read client info/request
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_read(success);tcp_readfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en read_tcp(char client_message[] )
{
	tcpstate_en Return_st=tcp_readfailed;
    int read_size;
	if((read_size = recv(client_sock ,client_message , 200 , 0)) != 0 )
	{
		Return_st=tcp_read;

	}
	return (Return_st);
}
/***********************************************************************************************************************************************/
/*
 * API				: To send data on TCP
 * Arguments 		: int sock, char buf[] , socket to send data on , Buffer of data to be sent
 * Function Usage 	: TCP read
 * Return Type 		: tcpstate_en
 * Return Value 	: tcp_send(success);tcp_sendfailed(Failure)
 *
 ***********************************************************************************************************************************************/
tcpstate_en send_tcp(int sock, char buf[] )
{
	tcpstate_en Return_st=tcp_send;int errval=0;
	int state;
	state=send(sock , buf , strlen(buf),MSG_NOSIGNAL);
	errval=errno;
	if(state < 0)
	{
		if((errval==EPIPE)|(errval==ECONNRESET))
		{
			errval =0;
			printf("TCP Send failed");
			Return_st=tcp_sendfailed;
		}
	}

	else
	{
		Return_st=tcp_send;
	}
	return (Return_st);
}
/***********************************************************************************************************************************************/




















