/*
 * main.c
 *
 *  Created on: Jan 3, 2019
 *      Author: bharath
 *
 */

/**************************************************Include Section **************************************************************************/
#include "stdio.h"
#include "string.h"
#include "sys/socket.h"
#include "arpa/inet.h"
#include "unistd.h"
#include "errno.h"
#include "fcntl.h"
#include "termios.h"
#include "stdlib.h"
#include <sys/types.h>
#include "time.h"
/**************************************************Global Variables *************************************************************************/
unsigned char buf;				/*Buffer to read USB one byte */



typedef enum
{
	False,
	True
}boolean;           			/*Data type for boolean*/



#define tcpport  9001  		/*35162 TCP port number*/
int sock;						/*Socket variable*/


typedef enum {
	state_init,
	state_main,
	state_err
}statetranstion_en;				/*Main function Enums*/

statetranstion_en Currentst;

typedef enum{
	usb_init,
	usb_open,
	usb_openfailed,
	usb_atset,
	usb_atsetfailed,
	usb_read,
	usb_readfailed
}usbstate_en;					/*USB status*/

typedef struct {

	usbstate_en usbsts;

}usbstatus_st;

usbstatus_st  usbstatus ;

int tcp_connectionretry;
typedef enum{
	tcp_init,
	tcp_socketcreated,
	tcp_socketnotcreated,
	tcp_connected,
	tcp_connectionfailed,
	tcp_send,
	tcp_sendfailed
}tcpstate_en;					/*TCP status*/

typedef struct {

	tcpstate_en tcpsts;

}tcpstatus_st;

tcpstatus_st  tcpstatus ;
FILE *Err;
/********************************************************************************************************************************************/

/********************************************************************************************************************************************/
/*Declaration Section*/

void usbinit();
void usb_statemachine();
usbstate_en usb_openport();
usbstate_en usb_setat();
int tcp_main(unsigned char buf[]);
void tcpinit();
void tcp_statemachine();
tcpstate_en create_socket();
tcpstate_en connect_tcp();
int usb_main(int file_st);
boolean clean_buf();
void file_initilization();
void filewrite(unsigned char *buf[]);
/********************************************************************************************************************************************/

/******Macros *******************************************************************************************************************************/
#define OpenPort "/dev/ttyUSB0" /*Listen to USB */
/********************************************************************************************************************************************/

int main()
{
	printf("Serial Communication on Raspberry-Pi and SHH TCP Connection:\n");

	while (1)
	{
		switch (Currentst)
		{
		case state_init : tcpinit();
		                  usbinit();

		                  Currentst=state_main;
		break;

		case state_main : tcp_statemachine();
		                  usb_statemachine();
		                  break;

		case state_err  : break;                 /*To be used in future or any special err function handling*/

		default : break;
		}

	}
return 0;
}
/********************************************************************************************************************************************/
struct sockaddr_in  server;
void tcpinit()
{
	printf("Initialization:\n");
    close(sock);
    server.sin_addr.s_addr = inet_addr("127.0.0.1"); /*"83.220.137.136"IP address of the server*/
	server.sin_family = AF_INET;
	server.sin_port = htons(tcpport);
	tcpstatus.tcpsts= tcp_init;

	return;

}
/********************************************************************************************************************************************/
void tcp_statemachine()
{

	switch(tcpstatus.tcpsts)
	{

	case tcp_init          		: tcpstatus.tcpsts=create_socket();
	                              break;
	case tcp_socketcreated 		: tcpstatus.tcpsts=connect_tcp();
	                         	  break;
	case tcp_socketnotcreated 	: tcpstatus.tcpsts = tcp_init;
	                              break;
	case tcp_connected          : tcpstatus.tcpsts = tcp_send;
	                              break;
	case tcp_connectionfailed   : tcpstatus.tcpsts = tcp_socketcreated;
	                              break;

	case tcp_sendfailed         : tcpstatus.tcpsts = tcp_init;
                                  close(sock);
                                  break;


	default : break;


	}
}
/********************************************************************************************************************************************/
tcpstate_en create_socket()
{

	tcpstate_en Return_st=tcp_socketcreated;
	sock = socket(AF_INET , SOCK_STREAM , 0);
    if(sock== -1)
    {


    	Return_st=tcp_socketnotcreated;
    	printf("Could not create socket\n");

    }
    else
    {
    	printf("Socket created\n");
    }

return Return_st ;
}
/********************************************************************************/
tcpstate_en connect_tcp()
{
	tcpstate_en Return_st=tcp_connected;
	if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
	{

		printf("connect failed. Error\n");
		Return_st=tcp_connectionfailed;


	}
	else
	{
		printf("Connected\n");
	}

	return Return_st;
}

/********************************************************************************************************************************************/
int cnt=0;
int tcp_main( unsigned char buffer[])
{
	unsigned char Errmsg[]="TCP Send Failed\n";
	unsigned char val1[15]={0};
	unsigned char val2[5]={0};
	unsigned char timervalue[255]={0};
	struct timespec timerval;
	unsigned long int sec=0,ms=0;
	unsigned char *ptr=NULL;
	unsigned int len=0;
	unsigned char *ptr1=NULL;int len1=0;
	//Send some data

	clock_gettime(CLOCK_REALTIME,&timerval);

	sec= timerval.tv_sec;
	ms = (timerval.tv_nsec) / 1000000;

	sprintf(val1,"%ld",sec);
	sprintf(val2,"%ld",ms);
	if(ms<100)
	{
		if(ms<10)
		{
			ptr1=val1;
			len1 = strlen(val1);
			ptr1+=len1;
			*ptr1='0';
			ptr1+=1;
			*ptr1='0';
			
		}

		else
		{
	    ptr1=val1;
		len1 = strlen(val1);
		ptr1+=len1;
		*ptr1='0';
		}

	}

	strcat(val1,val2);
	strcpy(timervalue,val1);

	ptr =timervalue;
	len = strlen(timervalue);
	ptr+=len;

	*ptr='|';
	strcat(timervalue,buffer);
        if((timervalue[28]=='4')& (cnt<100))
	{
	  filewrite(timervalue);
        }
	printf("%s",timervalue);
	if( send(sock , timervalue , strlen(timervalue), 0) < 0)
	{
		printf("TCP Send failed");
		filewrite(Errmsg);
		tcpstatus.tcpsts=tcp_sendfailed;

	}
	else
	{
		tcpstatus.tcpsts=tcp_send;


	}


	return 0;

}

/********************************************************************************************************************************************/
int comport=0;
/********************************************************************************************************************************************/
void usbinit()
{
	printf("Serial Communication on Raspberry Pi:\n");
	usbstatus.usbsts=usb_init;
	close(comport);
}

/********************************************************************************************************************************************/

void usb_statemachine()
{

	switch(usbstatus.usbsts)
	{
	case usb_init 			: usbstatus.usbsts = usb_openport();
	                          break;
	case usb_open 			: usbstatus.usbsts = usb_setat();
	                          break;
	case usb_openfailed 	: usbstatus.usbsts=usb_init;
	                          break;
	case usb_atset      	: usbstatus.usbsts =usb_read;
	                          usb_main(comport);
		                      break;
	case usb_atsetfailed 	: usbstatus.usbsts = usb_open;
	                          break;

	case usb_readfailed 	: usbstatus.usbsts=usb_init;
		                      close(comport);
                              while(usb_openport()!=usb_open){;}
                              close(comport);

	                          break;

	default 				: break;
	}
}

/********************************************************************************************************************************************/
usbstate_en usb_openport()
{
	usbstate_en Return=usb_open;
	unsigned char device[20]="/dev/ttyUSB";
	unsigned char d_number[3];
    int x=0,i=0;
    sleep(20);
	x = strlen(device);
    for(i=0;i<8;i++)
		{
    	 printf("Loop no. %d\n",i);
         sprintf(d_number,"%d",i);
	     device[x] = '\0';
	     strcat(device,d_number);
         if((comport =open(device, O_RDWR | O_NOCTTY | O_NDELAY))>0)
	       {
             printf("Port opened\n");
             Return = usb_open;
        	 break;

	       }
         else
         {
        	 printf("Connecting to ttyUSB%d\n",i+1);
        	 Return = usb_openfailed;
         }

		}
    printf("%d\t%s\n",i,device);
	return Return;

}
/*******************************************************************************************************************************************/
usbstate_en usb_setat()
{
	usbstate_en Return =usb_atset;
	struct termios SerialPortSettings;	/* Create the structure                          */
	fcntl(comport, F_SETFL, 0);
	tcgetattr(comport, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings,B38400); /* Set Read  Speed as 38400*/

	SerialPortSettings.c_cflag &= ~PARENB;    /*Disables the Parity Enable bit(PARENB),So No Parity*/
	SerialPortSettings.c_cflag &= ~CSTOPB;    /*CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit*/
	SerialPortSettings.c_cflag &= ~CSIZE;	  /*Clears the mask for setting the data size*/
	SerialPortSettings.c_cflag |= CS8;        /*Set the data bits = 8*/
	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines*/


	//SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);         /*  Disable XON/XOFF flow control both i/p and o/p*/
	//SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);   /* Non Cannonical mode*/
        SerialPortSettings.c_iflag =0;
	SerialPortSettings.c_lflag = 35376;
	SerialPortSettings.c_oflag &= ~OPOST; /*No Output Processing*/

	/* Setting Time outs*/
	SerialPortSettings.c_cc[VMIN] = 255;  /*Read at least 10 characters*/
	SerialPortSettings.c_cc[VTIME] = 0;   /*Wait indefinetly*/
	if((tcsetattr(comport,TCSANOW,&SerialPortSettings)) != 0)  /* Set the attributes to the termios structure */
	{
		printf("\n  ERROR ! in Setting attributes");
		Return =usb_atsetfailed;
	}
	else
	{
		printf("\n  BaudRate = B38400 \n  StopBits = 1 \n  Parity   = none\n");
	}

	return Return;

}

/********************************************************************************************************************************************/
unsigned char tcpbuf[155]={0};

int usb_main(int file_st)
{

	unsigned char Errmsg[]="USB read failed:USB port not connected or no data\n";
    int i=0;
	ssize_t readmsg;

	/*Read the data from the port and put it in the buffer*/


	while((readmsg=read(file_st,&buf,1))!=0)
	{
		tcpbuf[i]=buf;
		i++;
		if(buf == '\n')
		{
		   buf=0;
           tcp_main(tcpbuf);
           

           clean_buf();
           printf("\n");
		   i=0;

		}

		printf("%c",buf);

		usbstatus.usbsts=  usb_read;
	}

	usbstatus.usbsts=usb_readfailed;
	clean_buf();



	return 0;
}
/***********************************************************************************************************************************************/
boolean clean_buf(){
	boolean Ret= True;
	buf=0;
	int j;
   for (j=0;j<155;j++)
   {
	   tcpbuf[j]=0;

   }

	return Ret;
}

/***********************************************************************************************************************************************/

void file_initilization()
{
	Err=fopen("/etc/config/scripts/Errlog.txt","w+");
    fprintf(Err,"******************Error Log**************************\n");
    fclose(Err);

}
/***********************************************************************************************************************************************/
void filewrite(unsigned char *buf[])
{
	Err=fopen("/etc/config/scripts/Errlog.txt","a");
	fprintf(Err,buf);
    fclose(Err);

}
/***********************************************************************************************************************************************/





























