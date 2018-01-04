#include "motion.h"
#include "timer.h"
#include "common.h"
#include<stdint.h>

#if defined(__AVR__) || defined(ESP8266)
    
#else
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>

void demo();

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
      xprintf(PSTR("Graphics error: %s\n"), grapherrormsg(errorcode));
      xprintf(PSTR("Press any key to halt:"));

      getch();
      exit(1);               /* terminate with an error code */
   }
	xprintf(PSTR("Simple Motion Control with Acceleration, Jerk, and lookahead planner\n"));
	xprintf(PSTR("By ryannining@gmail.com\n"));
	initmotion();
	setcolor(1);
	line(0,400,600,400);
	line(0,400-50*fscale,600,400-50*fscale);
	line(0,400-100*fscale,600,400-100*fscale);

    //int8_t z=100;
	float v=10.1234;
	xprintf (PSTR("F %f D %d\n"),ff(v),(int32_t)200);
  demo();            
	xprintf (PSTR("WAIT\n"));
	waitbufferempty();	
	xprintf (PSTR("Time:%f\n"),ff(tick/timescale));
	getch(); 
}
#endif

void demo(){
    int f=100;
	//addmove(f,0,50,0);
    int i;
    for (i=0;i<5;i++) {
        addmove(f,i*10,50,0);
    }
    addmove(f,60,55,0);
    for (i=5;i>0;i--) {
        addmove(f,i*10,60,0);
    }
    addmove(f,0,65,0);
    
}

