/*
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include "string.h"


int main(){
	    unsigned char Errmsg[]="TCP Send Failed\n";
	    unsigned char val1[50]={0};
	    unsigned char val2[5]={0};
	    unsigned char timervalue[255]={0};
	    struct timespec timerval;
	    unsigned long int sec=0,ms=0;
	    unsigned char *ptr=NULL; unsigned int len=0;
	    unsigned char *ptr1=NULL;int len1=0;

for(;;){
	    clock_gettime(CLOCK_REALTIME,&timerval);

	    sec= timerval.tv_sec;
	    ms = (timerval.tv_nsec)/1000000;
        sprintf(val1,"%ld",sec);
	    sprintf(val2,"%ld",ms);

	    if(ms<100)
	    {
	    	ptr1=val1;
            len1 = strlen(val1);
            ptr1+=len1;
            *ptr1='0';
            printf("%s\t ms<100 \t",val1);
	    }
	    if(ms<10)
	   	{
	   	    	ptr1=val1;
	               len1 = strlen(val1);
	               ptr1+=len1;
	               *ptr1='0';
	               ptr1+=1;
	               *ptr1='0';
	               printf("%s\t ms<10 \t",val1);

	   	 }

	    strcat(val1,val2);
	    strcpy(timervalue,val1);

	    ptr =timervalue;
	    len = strlen(timervalue);
	    ptr+=len;

	    *ptr='|';

	    printf("%s\n",timervalue);
}
	return 0;
}
*/
























































/* ============================================================================
 Name        : timer.c
 Author      : Bharath
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>     // for pause

#include <pthread.h>
#define timeout 800       //  number of milliseconds to go off

// function prototype
int pthread_create (pthread_t *thread,const pthread_attr_t *attr,void *(*start_routine) (void *),void *arg);
void thr(void);
void process(void);

int main() {

  struct itimerval it_val;  // for setting itimer

  /* Upon SIGALRM, call DoStuff().
   * Set interval timer.  We want frequency in ms,
   * but the setitimer call needs seconds and useconds.*/

  if (signal(SIGALRM, (void (*)(int)) thr) == SIG_ERR) {
    perror("Unable to catch SIGALRM");
    exit(1);
  }
  it_val.it_value.tv_sec =     (timeout/1000) ;
  it_val.it_value.tv_usec =    (timeout*1000) % 1000000;
  it_val.it_interval = it_val.it_value;
  if (setitimer(ITIMER_REAL, &it_val, NULL) == -1) {
    perror("error calling setitimer()");
    exit(1);
  }

  while (1)
    pause();
return 0;
}


// DoStuff

void thr(void) {
	  pthread_t th1;
	  pthread_create(&th1,NULL,(void*)*process,NULL);
      printf("\n");

}

void process(void){
	int i, firstlen=0,lastlen=0;
			unsigned char buf[]={"!AIVDM,1,1,,B,139a6QPP01PeU7<N`p8hj?vp00SA,0*3D"};

	      //  printf("%d\n",sizeof(buf));
			firstlen=14;
			lastlen=sizeof(buf)-6;



		for(i=firstlen;i<lastlen;i++)
			 printf("%c",buf[i]);

			return ;



}















































