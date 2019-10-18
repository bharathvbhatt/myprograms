/***********************************************************************************************************************************************/
/*
 * File name	: usbserial.c
 * Created on	: Feb 28, 2019
 * Author		: Bharath Vikram Bhatt <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#include "file_handling.h"
FILE *Err;
/***********************************************************************************************************************************************/
/*
 * API				: File initialization
 * Arguments 		: None
 * Function Usage 	: To initialize the file that is used to send HTTP AIS data
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void file_initilization()
{
	Err=fopen("/etc/config/scripts/ais.txt","w+");
    fclose(Err);
}
/***********************************************************************************************************************************************/
/*
 * API				: File write
 * Arguments 		: char buf[] ,string buffer that has to be written to the file
 * Function Usage 	: To data sent by the program to write into the file
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void filewrite( char buf[])
{
	Err=fopen("/etc/config/scripts/ais.txt","a");
	fprintf(Err,buf);
    fclose(Err);
}
/***********************************************************************************************************************************************/
/*
 * API				: File erase
 * Arguments 		: none
 * Function Usage 	: To data in the file and update with new data
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void file_erase()
{
  fclose(fopen("/etc/config/scripts/ais.txt","w"));
}
/***********************************************************************************************************************************************/





