/*
 ============================================================================
 Name        : http_file.c
 Author      : Bharath
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <sys/sendfile.h>
#include "errno.h"
#include "fcntl.h"
#include "termios.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <netinet/tcp.h>

int client_sock;

unsigned char header[100]="HTTP/1.1 200 OK\nContent-Type: text";//html\nContent-Length: ";

struct stat buf;
int state=1;
int main(void) {


	int socket_desc , c , read_size;
	struct sockaddr_in server , client;
	unsigned char client_message[200];
	int fp;
	unsigned char clen[8];
	int f_st, set_st,f_sts;

	//char *hello = "HTTP/1.1 200 OK\nContent-Type: text/html\nContent-Length: 44\n\n<html><body><h1>It works!</h1></body></html>";
	char *error = "HTTP/1.1 404 NOT FOUND";
	printf("Server is running....\n");
	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
				printf("Could not create socket\n");
	}
	printf("Socket created\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons( 8080 );

	//Bind
	if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
	{
		//print the error message
		perror("bind failed. Error");
		return 1;
	}
		printf("bind done\n");

	//Listen
	listen(socket_desc , 3);

	//Accept and incoming connection
		printf("Waiting for incoming connections...\n");
	c = sizeof(struct sockaddr_in);





	while(1){
		//accept connection from an incoming client
		client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
		if (client_sock < 0)
		{
			perror("accept failed");
			return 1;
		}


		while( (read_size = recv(client_sock ,client_message , 200 , 0)) != 0 )
		{
			//Send the message back to client
			//write(client_sock , client_message , strlen(client_message));
			printf("%s\n",client_message);

			if(strstr(client_message,"GET /myhtml HTTP/1.1")!=NULL)

			{
				strcat(header,"HTTP/1.1 200 OK\nContent-Type: text");
				fp = open("/etc/config/scripts/javascript.html",O_RDONLY);
				if(fp<0)
				{
					printf("Error file cant be opened!\n");
				}
				strcat(header,"/html\nContent-Length: ");
				fstat(fp,&buf);
				sprintf(clen,"%d",buf.st_size);
				strcat(header,clen);
				strcat(header,"\n");
				strcat(header,"\n");
				set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
				if(set_st < 0){printf("Error! setoptsock\n");}
				f_st=send(client_sock , header ,strlen(header),0);if( f_st< 0){printf("Error! setoptsock\n");}
				f_sts=sendfile(client_sock,fp,NULL,buf.st_size);if( f_sts< 0){printf("Error! setoptsock\n");}
				state=0;
				setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
				if(set_st < 0){printf("Error! setoptsock\n");}

			}


			else if(strstr(client_message,"GET /myScript.js HTTP/1.1")!=NULL)
			{

                strcat(header,"HTTP/1.1 200 OK\nContent-Type: text");
				fp = open("/etc/config/scripts/myScript.js",O_RDONLY);
				if(fp<0)
				{
					printf("Error file cant be opened!\n");
				}
				strcat(header,"/javascript\nContent-Length: ");
				fstat(fp,&buf);
				sprintf(clen,"%d",buf.st_size);
				strcat(header,clen);
				strcat(header,"\n");
				strcat(header,"\n");
				set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
				if(set_st < 0){printf("Error! setoptsock\n");}
				f_st=send(client_sock , header ,strlen(header),0);if( f_st< 0){printf("Error! setoptsock\n");}
				f_sts=sendfile(client_sock,fp,NULL,buf.st_size);if( f_sts< 0){printf("Error! setoptsock\n");}
				state=0;
				setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
				if(set_st < 0){printf("Error! setoptsock\n");}
			}



			else if(strstr(client_message,"GET /test HTTP/1.1")!=NULL)
			{

				strcat(header,"HTTP/1.1 200 OK\nContent-Type: text");
				fp = open("/etc/config/scripts/test.txt",O_RDONLY);
				if(fp<0)
				{
					printf("Error file cant be opened!\n");
				}
				strcat(header,"/plain\nContent-Length: ");
				fstat(fp,&buf);
				sprintf(clen,"%d",buf.st_size);
				strcat(header,clen);
				strcat(header,"\n");
				strcat(header,"\n");
				set_st =  setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
				if(set_st < 0){printf("Error! setoptsock\n");}
				f_st=send(client_sock , header ,strlen(header),0);if( f_st< 0){printf("Error! setoptsock\n");}
				f_sts=sendfile(client_sock,fp,NULL,buf.st_size);if( f_sts< 0){printf("Error! setoptsock\n");}
				state=0;
				setsockopt(client_sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
				if(set_st < 0){printf("Error! setoptsock\n");}
			}


			memset(client_message,'\0',strlen(client_message));
			memset(header,'\0',strlen(header));


		}


		}




		memset(client_message,'\0',strlen(client_message));
		close(fp);
		return 0;
}





