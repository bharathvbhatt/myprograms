/***********************************************************************************************************************************************/
/*
 * File name	: datastruct.h
 * Created on	: June 19, 2019
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#include <string.h>
#include <stdio.h>


/***********************************************************************************************************************************************/
/*
 * API				: get_aisdata , A ringbuffer Implementation
 * Arguments 		: char aisdata[],string of AIS data sent
 * Function Usage 	: Ring buffer that stores the AIS stream/string which is then used for HTTP file send
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void get_aisdata(char aisdata[]);
/***********************************************************************************************************************************************/
/*
 * API				: file_get ,  write the file with ring buffer data
 * Arguments 		: none
 * Function Usage 	: when HTTP request for AIS data is received, the ringbuffer data is dumped into text file.
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void file_get();
