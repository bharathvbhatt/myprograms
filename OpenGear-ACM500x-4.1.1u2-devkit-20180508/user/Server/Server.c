/*
 * Server.c
 *
 *  Created on: Dec 27, 2018
 *      Author: bharath
 */


#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>

int main(int argc , char *argv[])
{
	int socket_desc , client_sock , c , read_size;
	struct sockaddr_in server , client;
	unsigned char client_message[200];
     
   char *hello = "HTTP/1.1 200 OK\nContent-Type: text/plain\nContent-Length: 12\n\nHello world!";
	
       //Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}
	puts("Socket created");

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
	puts("bind done");

	//Listen
	listen(socket_desc , 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");
	c = sizeof(struct sockaddr_in);

	//accept connection from an incoming client
	client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
	if (client_sock < 0)
	{
		perror("accept failed");
		return 1;
	}
	puts("Connection accepted");

	//Receive a message from client
	while( (read_size = recv(client_sock ,client_message , 200 , 0)) != 0 )
	{
		//Send the message back to client
		//write(client_sock , client_message , strlen(client_message));
		printf("%s",client_message);
             write(client_sock , hello, strlen(hello));
	}
        
       
       

//	if(read_size == 0)
	//{
	//	puts("Client disconnected");
	//	fflush(stdout);
//	}
	//else if(read_size == -1)
	//{
	//	perror("recv failed");
	//}

	return 0;
}

