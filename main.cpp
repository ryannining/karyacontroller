#include "motion.h"
#include "timer.h"
#include "common.h"
#include<stdint.h>

#ifdef ISPC

#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>
#include "gcode.h"


void demo();
void demofile();


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
  graphscale=25;
  line(0, 400, 600, 400);
  line(0, 400 - 50 * fscale, 600, 400 - 50 * fscale);
  line(0, 400 - 100 * fscale, 600, 400 - 100 * fscale);

  //int8_t z=100;
  float v = 10.1234;
  xprintf (PSTR("F %f D %d\n"), ff(v), (int32_t)200);
  demofile();
  //demo();
  xprintf (PSTR("WAIT\n"));
  waitbufferempty();
  xprintf (PSTR("Time:%f\n"), ff(tick / timescale));
  getch();
}
void demofile(){
    FILE *file = fopen("d:/git/hipopotamo.gcode", "r");
    char code[100];
    size_t n = 0;
    int c;
    graphscale=5;
    if (file == NULL) return; //could not open file
    int comment=0;
    while ((c = fgetc(file)) != EOF) {
        if (c==';')comment=1;
        code[n++] = (char) c;
        if (c=='\n'){
            code[n++]=0;
            printf("\n%s",code);
            if (!comment)gcode_parse_char(c);
            n=0;
            comment=0;
            //getch();
        } else 
        {
            if (!comment)gcode_parse_char(c);

        }
    }

    
 }

#endif

void demo() {
  graphscale=1;  
  int f = 40;
  addmove(f,0,50,0,1);
  int i;
/*  for (i = 0; i < 5; i++) {
    addmove(f, i * 10, 50, 0, 0);
  }
  addmove(f, 60, 55, 0, 0);
  for (i = 5; i > 0; i--) {
    addmove(f, i * 10, 60, 0, 0);
  }
  addmove(f, 0, 65, 0, 0);
  for (i = 0; i < 36; i++) {
    addmove(f, sin(i * 44.0/7/36)*10,  -cos(i * 44.0/7/36)*10, 0, 1);
  }
*/
  

}


 

