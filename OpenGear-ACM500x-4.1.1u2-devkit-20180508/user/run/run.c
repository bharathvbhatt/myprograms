/*
 ============================================================================
 Name        : run.c
 Author      : Bharath
 Version     : 001
 Copyright   : Your copyright notice
 Description :StartUp program for Serial communication for AIS data reception, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>

unsigned int startupvar =0 ;
int i=0;
int main(void) {

	 if(startupvar==0)
	 {
           i++;
           system("/etc/config/scripts/AIS");
           startupvar=1;
           printf("%d",i);

	 }

     return 0;
}
