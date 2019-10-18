/***********************************************************************************************************************************************/
/*
 * File name	: tcp.c
 * Created on	: Dec 20, 2018
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#include "time.h"
#include "tcp_driver.h"
#include "tcp_main.h"
#include <pthread.h>
/***********************************************************************************************************************************************/
sockaddr sockdata_table[max_data],client;
pthread_t th1,th0;
tcpstatus_st  tcpstatus[max_data] ;
int sock[max_data];
extern int client_sock;
serverlist data[max_data] =
{
	  /*{9999,"10.110.9.67"}, */   	    /*Client1*/
		{8888,"10.110.9.67"},	        /*Client2*/
		{8080,"00.00.00.00"}, 		    /*Server*/
};

/***********************************************************************************************************************************************/
/*
 * API				: TCP int Function
 * Arguments 		: none
 * Function Usage 	: TCP socket creation and initialization
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void init_tcp()
{
	int i=0;

	for(i=0; i< max_data;i++)  /*For all the data in table */
	{
		tcpstatus[i].tcpsts = tcpinit(&sock[i],&sockdata_table[i],data[i].name,data[i].port); /*Create socket*/


		if((tcpstatus[i].tcpsts==tcp_socketcreated)) /*If socket created*/
		{
			ioctl(sock[i], FIONBIO, 1);              /*set all opened socket into Non-Blocking mode*/
			tcpstatus[i].tcpsts=connect_tcp(sock[i], sockdata_table[i]); /*Connect the sockets*/
		}

	}

	server_init();

	return;

}
/***********************************************************************************************************************************************/
/*
 * API				: TCP Clientfault handling
 * Arguments 		: void
 * Function Usage 	: To handle Connect lost and other small socket faults(Detailed error handling is not done)
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/

void tcp_Clientfaulthandling()
{
	int i=0;
	/*Client fault Handling*/
	for(i=0;i< max_data-1;i++)
	{
		switch(tcpstatus[i].tcpsts)
		{

		case tcp_socketnotcreated  :  tcpstatus[i].tcpsts = tcpinit(&sock[i],&sockdata_table[i],data[i].name,data[i].port); /* Create a socket*/
		                              tcpstatus[i].tcpsts = connect_tcp(sock[i], sockdata_table[i]); /*Connect the created socket*/
		                              break;
		case tcp_sendfailed        :  close(sock[i]);
		                              tcpstatus[i].tcpsts=tcp_socketnotcreated;
		                              break;
		case tcp_connectionfailed  :  tcpstatus[i].tcpsts = connect_tcp(sock[i], sockdata_table[i]);
		                              if(tcpstatus[i].tcpsts==tcp_connectionfailed)
		                                  {
		      	                            close(sock[i]);
			                                tcpstatus[i].tcpsts=tcp_socketnotcreated;
		                                  }
		                              break;
		default                    :  break;
		}
	}
}
/***********************************************************************************************************************************************/
/*
 * API				: TCP Serverfault handling
 * Arguments 		: void
 * Function Usage 	: To handle Connect lost and other small socket faults(Detailed error handling is not done)
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void tcp_Serverfaulthandling()
{
	switch(tcpstatus[server_web].tcpsts)
	{
	case tcp_socketnotcreated : tcpstatus[server_web].tcpsts = tcpinit(&sock[server_web],&sockdata_table[server_web],data[server_web].name,data[server_web].port);
	                            server_init();
	                            break;
	case tcp_bindfailed       : close(sock[server_web]);

	                            tcpstatus[server_web].tcpsts=tcp_socketnotcreated;
	                            break;
	default                   : break;
	}
}

/***********************************************************************************************************************************************/
/*
 * API				: TCP main function
 * Arguments 		: char tcpbuf[], contains the AIS data to be sent to the server (Client Function)
 * Function Usage 	: To send the TCP data to server.
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void tcp_main(char tcpbuf[])
{
	int i=0;
	/*tcpstate_en state=tcp_send;*/
	for(i=0;i< (max_data-1);i++)
	{
		if((tcpstatus[i].tcpsts==tcp_connected)|(tcpstatus[i].tcpsts==tcp_send))
		{

			tcpstatus[i].tcpsts =send_tcp(sock[i],tcpbuf);
		}
	}
}
/***********************************************************************************************************************************************/
/*
 * API				: TCP server initialization
 * Arguments 		: void
 * Function Usage 	: Init of server, bind and listen functionality is taken care.
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void server_init()
{

	tcpstatus[server_web].tcpsts=tcp_bind;
	if((tcpstatus[server_web].tcpsts= bind_tcp(sock[server_web],sockdata_table[server_web]))!=tcp_bind)
	{
		printf("Bind Error\n");
		tcpstatus[server_web].tcpsts=tcp_bindfailed;
	}

	listen(sock[server_web],5);
#ifdef	Server_software_cs
	thr();
#endif
}
/***********************************************************************************************************************************************/
/*
 * API				: Thread to handle the server implementation
 * Arguments 		: void
 * Function Usage 	: Thread to handle the server implementation
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void thr(void)
{
	pthread_create(&th1,NULL,(void*)*check_irq,NULL);
}
/***********************************************************************************************************************************************/
/*
 * API				: Server program to check IRQ
 * Arguments 		: void
 * Function Usage 	: TO check any client has requested for service
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void check_irq()
{
	tcpstate_en state=tcp_accept;
	char client_message[200];
	int read_size;
	printf("Thread 1 created\n");
	while((state=accept_tcp(sock[server_web], client)==tcp_accept))
	{
		printf("New request\n");
	    if((read_size = recv(client_sock ,client_message , 200 , 0)) != 0 )
		{
			/*Send the message back to client*/

			if(strstr(client_message,"GET /")!=NULL)
			{
				printf("%s\n",client_message);

                server_send(client_message);

			}
		}
	    usleep(50000);
	    close(client_sock);
	}
}
/***********************************************************************************************************************************************/
char header[100]={};//html\nContent-Length: ";
struct stat bufsize;
int state=1;
/***********************************************************************************************************************************************/
/*
 * API				: Main HTTP server implementation
 * Arguments 		: char buff[], consists the stream/string of HTTP request
 * Function Usage 	: TO check any client has requested for service
 * Return Type 		: void
 * Return Value 	: void
 *
 **********************************************************************************************************************************************/
void server_send( char buff[])
{
    int fp;
	char clen[8];
	int f_st, set_st,f_sts;
	printf("Thread 2 created\n");
	if(strstr(buff,"GET /index.html HTTP/1.1")!=NULL)
	{
		strcat(header,"HTTP/1.1 200 OK\nContent-Type: text");
		fp = open("/etc/config/scripts/index.html",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/html\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
	else if(strstr(buff,"GET /styles.ff4089ceb584020db346.css HTTP/1.1")!=NULL)
	{

		strcat(header,"HTTP/1.1 200 OK\nContent-Type: text/css;charset=utf-8");
		fp = open("/etc/config/scripts/styles.ff4089ceb584020db346.css",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/javascript\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
	else if(strstr(buff,"GET /runtime.2f29e12616932f0ed037.js HTTP/1.1")!=NULL)
	{
		strcat(header,"HTTP/1.1 200 OK\nContent-Type: application/javascript;charset=utf-8");
		fp = open("/etc/config/scripts/runtime.2f29e12616932f0ed037.js",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/javascript\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
	else if(strstr(buff,"GET /polyfills.d480d2ebf3b3121ca487.js HTTP/1.1")!=NULL)
	{

		strcat(header,"HTTP/1.1 200 OK\nContent-Type: application/javascript;charset=utf-8");
		fp = open("/etc/config/scripts/polyfills.d480d2ebf3b3121ca487.js",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/javascript\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
	else if(strstr(buff,"GET /main.b72a0f7abd963372e58d.js HTTP/1.1")!=NULL)
	{
		strcat(header,"HTTP/1.1 200 OK\nContent-Type: application/javascript;charset=utf-8");
		fp = open("/etc/config/scripts/main.b72a0f7abd963372e58d.js",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/javascript\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
	else if(strstr(buff,"GET /vendor.js HTTP/1.1")!=NULL)
		{

			strcat(header,"HTTP/1.1 200 OK\nContent-Type: application/javascript;charset=utf-8");
			fp = open("/etc/config/scripts/vendor.js",O_RDONLY);
			if(fp<0)
			{
				printf("Error file cant be opened!\n");
			}
			strcat(header,"/javascript\nContent-Length: ");
			fstat(fp,&bufsize);
			sprintf(clen,"%ld",bufsize.st_size);
			strcat(header,clen);
			strcat(header,"\n");
			strcat(header,"\n");
			set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
			if(set_st < 0)
			{
				printf("Error! setoptsock\n");
			}
			f_st=send(client_sock , header ,strlen(header),0);
			if( f_st< 0)
			{
				printf("Error! setoptsock\n");
			}
			f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
			if( f_sts< 0)
			{
				printf("Error! setoptsock\n");
			}
			state=0;
			setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
			if(set_st < 0)
			{
				printf("Error! setoptsock\n");
			}
		}
	else if(strstr(buff,"GET /layers-2x.4f0283c6ce28e888000e.png HTTP/1.1")!=NULL)
	{
		strcat(header,"HTTP/1.1 200 OK\nContent-Type: image/x-icon");
		fp = open("/etc/config/scripts/layers-2x.4f0283c6ce28e888000e.png",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/plain\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
	else if(strstr(buff,"GET /ais HTTP/1.1")!=NULL)
	{
		printf("Getting data text\n");
		file_get();
		strcat(header,"HTTP/1.1 200 OK\nContent-Type: text");
		fp = open("/etc/config/scripts/ais.txt",O_RDONLY);
		if(fp<0)
		{
			printf("Error file cant be opened!\n");
		}
		strcat(header,"/plain\nContent-Length: ");
		fstat(fp,&bufsize);
		sprintf(clen,"%ld",bufsize.st_size);
		strcat(header,clen);
		strcat(header,"\n");
		strcat(header,"\n");
		set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
		f_st=send(client_sock , header ,strlen(header),0);
		if( f_st< 0)
		{
			printf("Error! setoptsock\n");
		}
		f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
		if( f_sts< 0)
		{
			printf("Error! setoptsock\n");
		}
		state=0;
		setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
		if(set_st < 0)
		{
			printf("Error! setoptsock\n");
		}
	}
/*.........................................................................................................................*/
	else if(strstr(buff,"GET /layers.a6137456ed160d760698.png HTTP/1.1")!=NULL)
		{
			printf("Getting data text\n");
			file_get();
			strcat(header,"HTTP/1.1 200 OK\nContent-Type: image/x-icon");
			fp = open("/etc/config/scripts/layers.a6137456ed160d760698.png",O_RDONLY);
			if(fp<0)
			{
				printf("Error file cant be opened!\n");
			}
			strcat(header,"/plain\nContent-Length: ");
			fstat(fp,&bufsize);
			sprintf(clen,"%ld",bufsize.st_size);
			strcat(header,clen);
			strcat(header,"\n");
			strcat(header,"\n");
			set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
			if(set_st < 0)
			{
				printf("Error! setoptsock\n");
			}
			f_st=send(client_sock , header ,strlen(header),0);
			if( f_st< 0)
			{
				printf("Error! setoptsock\n");
			}
			f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
			if( f_sts< 0)
			{
				printf("Error! setoptsock\n");
			}
			state=0;
			setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
			if(set_st < 0)
			{
				printf("Error! setoptsock\n");
			}
		}
	else if(strstr(buff,"GET /marker-icon.2273e3d8ad9264b7daa5.png HTTP/1.1")!=NULL)
		{
			printf("Getting data text\n");
			file_get();
			strcat(header,"HTTP/1.1 200 OK\nContent-Type: image/x-icon");
			fp = open("/etc/config/scripts/marker-icon.2273e3d8ad9264b7daa5.png",O_RDONLY);
			if(fp<0)
			{
				printf("Error file cant be opened!\n");
			}
			strcat(header,"/plain\nContent-Length: ");
			fstat(fp,&bufsize);
			sprintf(clen,"%ld",bufsize.st_size);
			strcat(header,clen);
			strcat(header,"\n");
			strcat(header,"\n");
			set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
			if(set_st < 0)
			{
				printf("Error! setoptsock\n");
			}
			f_st=send(client_sock , header ,strlen(header),0);
			if( f_st< 0)
			{
				printf("Error! setoptsock\n");
			}
			f_sts=sendfile(client_sock,fp,NULL,bufsize.st_size);
			if( f_sts< 0)
			{
				printf("Error! setoptsock\n");
			}
			state=0;
			setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
			if(set_st < 0)
			{
				printf("Error! setoptsock\n");
			}
		}
	memset(buff,'\0',strlen(buff));
	memset(header,'\0',strlen(header));
	return;

}
/***********************************************************************************************************************************************/









