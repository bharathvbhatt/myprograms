/***********************************************************************************************************************************************
 *
 * File name	: datastruct.c
 * Created on	: June 12, 2019
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 ***********************************************************************************************************************************************/
#include "datastruct.h"
#include "file_handling.h"

#define Max_buf_len 100
static char AIS_Mainbuf[Max_buf_len][150]={{'\0'}};
static int ais_index=0;

/***********************************************************************************************************************************************
*
 * API				: get_aisdata , A ringbuffer Implementation
 * Arguments 		: char aisdata[],string of AIS data sent
 * Function Usage 	: Ring buffer that stores the AIS stream/string which is then used for HTTP file send
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void get_aisdata(char aisdata[])
{


	if (ais_index < Max_buf_len)
	{
		strcpy(AIS_Mainbuf[ais_index],aisdata);

	}
	else if(ais_index >= Max_buf_len)
	{
		ais_index=0;
		strcpy(AIS_Mainbuf[ais_index],aisdata);
	}
	ais_index++;


	return;
}
/***********************************************************************************************************************************************
 *
 * API				: file_get ,  write the file with ring buffer data
 * Arguments 		: none
 * Function Usage 	: when HTTP request for AIS data is received, the ringbuffer data is dumped into text file.
 * Return Type 		: void
 * Return Value 	: void
 *
 ***********************************************************************************************************************************************/
void file_get()
{
	int i=0;
    file_erase();
    for (i=0;i<Max_buf_len;i++)
    {
    	filewrite(AIS_Mainbuf[i]);
    }

}
/***********************************************************************************************************************************************/

