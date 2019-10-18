/***********************************************************************************************************************************************/
/*
 * File name	: main.cpp
 * Created on	: Dec 17, 2018
 * Author		: Bharath Vikram Bhatt  <bhatt@vesseltracker.com>
 *
 * Main function is defined here
 *
 ***********************************************************************************************************************************************/
#include "stdio.h" /*100th line*/

#include "tcp_driver.h"
#include "usb_init.h"
#include "tcp_main.h"
#include "file_handling.h"
/***********************************************************************************************************************************************/
/*
 * API				: Main function
 * Arguments 		: None
 * Function Usage 	: Codes the entire program state
 * Return Type 		: int
 * Return Value 	: 0 on success
 *
 ***********************************************************************************************************************************************/
statetranstion_en Currentst=state_init;
int main(int argc , char * argv[])
{
	printf("AIS Software .......!:\n");

	if(argc > 1)
	{
		if((strstr(argv,"-Version"))!=NULL)
		{
			printf("GIT VERSION - %s\n", GIT_VERSION);
		}
	}

	else{
		while (True)
		{

			switch (Currentst)
			{
			case state_init : usbinit();
			init_tcp();
			file_initilization();
			Currentst=state_main;
			break;

			case state_main : usb_statemachine();

			break;

			case state_err  :
				break;

			default         : break;
			}

		}
	}
	return (0);
}
/****************************************************************************************************************************************/
