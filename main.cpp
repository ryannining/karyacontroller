#if defined(__AVR__) || defined(ESP8266)
    
#else
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include "motion.h"
#include "timer.h"



int main(void)
{
 /* request autodetection */
   int gdriver = DETECT, gmode, errorcode;
   int x, y;

   /* initialize graphics and local variables */
   initgraph(&gdriver, &gmode, "");

   /* read result of initialization */
   errorcode = graphresult();
   if (errorcode != grOk) {  /* an error occurred */
      printf("Graphics error: %s\n", grapherrormsg(errorcode));
      printf("Press any key to halt:");

      getch();
      exit(1);               /* terminate with an error code */
   }
	printf("Simple Motion Control with Acceleration, Jerk, and lookahead planner\n");
	printf("By ryannining@gmail.com\n");
	initmotion();
	setcolor(1);
	line(0,400,600,400);
	line(0,400-50*fscale,600,400-50*fscale);
	line(0,400-100*fscale,600,400-100*fscale);

	         
	addmove(10,50,30,0);
	addmove(50,100,60,0);
	addmove(10,150,90,0);
	waitbufferempty();	
	printf ("Time:%f",tick);
	getch(); 
}
#endif

