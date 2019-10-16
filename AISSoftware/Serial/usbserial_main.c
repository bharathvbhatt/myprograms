/***********************************************************************************************************************************************/
/*
 * File name	: usbserial.c
 * Created on	: Dec 20, 2018
 * Author		: Bharath Vikram Bhatt <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
/*Add Serial open and read from the USB and all function needed for it.*/
#include "usb_init.h"
#include "usbserial_main.h"
#include "tcp_main.h"
#include "string.h"
#include "tcp_driver.h"
#include "datastruct.h"
#include "time.h"
 char msgbuf[155]={'\0'};
 char timerbuffer[155]={'\0'};
/***********************************************************************************************************************************************/
/*
 * API				: USB Main Function
 * Arguments 		: None
 * Function Usage 	: USB read
 * Return Type 		: Integer type
 * Return Value 	: 0 on successful execution
 *
 ***********************************************************************************************************************************************/
 int usb_main(int file_st)
 {


	 int i=0;
	 ssize_t readmsg;

	 /*Read the data from the port and put it in the buffer*/

	 while(((readmsg=read(file_st,&buf,1))!=0))
	 {
		 msgbuf[i]=buf;
		 i++;
		 /*Check if new line character encountered*/
		 if(buf == '\n')
		 {
			 buf=0;

        /*Send to TCP ; Client software*/
        /*Get time-stamp of the machine*/
           get_timerval(timerbuffer);
           strcat(timerbuffer,msgbuf);


#ifdef Client_software_cs
			 tcp_main(timerbuffer);
			 tcp_Clientfaulthandling();
#endif
         /*Server side fault check and Ring-buffer update for HTTP server*/
#ifdef Server_software_cs
			 tcp_Serverfaulthandling();
			 get_aisdata(timerbuffer);
#endif
			 /*Clean Buffer*/
			 clean_buf();
			 i=0;
			 usbstatus.usbsts=  usb_read;

		 }
		 /*Keep Status in Read*/
		 usbstatus.usbsts=  usb_read;
	 }

	 /*If read is failed update the status*/
	 usbstatus.usbsts=usb_readfailed ;

	 clean_buf();

	 return (0);
 }

/***********************************************************************************************************************************************
 *
 * API				: Clean Buffer
 * Arguments 		: None
 * Function Usage 	: Clean all buffers once the buffered data is sent over TCP
 * Return Type 		: boolean
 * Return Value 	: True
 *
 ***********************************************************************************************************************************************/
boolean clean_buf(){
	boolean Ret= True;
	buf=0;
	/*Clean the buffer*/
	memset(msgbuf,'\0',strlen(msgbuf));
	memset(timerbuffer,'\0',strlen(timerbuffer));
    return (Ret);
}
/************************************************************************************************************************************************
 *
 * API				: Get unix time-stamp
 * Arguments 		: char time[] : the refernce passing buffer with timer value
 * Function Usage 	: To get time-stamp
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void get_timerval(char time[])
{
	    struct timespec timerval;
		unsigned long int sec=0,ms=0;
	    int len1=0,len2=0;
	    char second[16]={'\0'};
	   	char milli[5]={'\0'};

		    clock_gettime(CLOCK_REALTIME,&timerval);

			sec= timerval.tv_sec;
			ms = (timerval.tv_nsec) / 1000000;
	        sprintf(second,"%ld",sec);
		    sprintf(milli,"%ld",ms);
		    if(ms<100)
		    	{
		    		if(ms<10)
		    		{
		    	     len1 = strlen(second);
		    		 second[len1]='0';
		    		 second[len1+1]='0';
		    		}
	         else
		    		{

	        	    len1 = strlen(second);
	    			second[len1]='0';

		    		}

		    	}
		    strcat(second,milli);
		    len2=strlen(second);
		    second[len2]='|';
	        strcpy(time,second);
}
/***********************************************************************************************************************************************/


