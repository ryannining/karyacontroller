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
  int gdriver =  DETECT, gmode, errorcode;
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
  xprintf(PSTR("Simple Motion Control with Acceleration, and lookahead planner\n"));
  xprintf(PSTR("By ryannining@gmail.com\n"));
  initmotion();
  struct palettetype pal; 
  //getpalette(&pal);
  for (int i=0;i<256;i++){
    //setrgbpalette(i,i,i,i);
  }
  setcolor(1);
  graphscale = 25;
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
void demofile() {
  //#define fn "d:/git/hipopotamo.gcode"
  //#define fn "d:/git/bowdenlock.gcode" 
  //#define fn "d:/3d/fish_fossilz.gcode"
  #define fn "d:/3d/cube20.gcode"

  //#define fn "d:/3d/box1cm.gcode" 
  FILE *file = fopen(fn,"r");
  char code[100];
  size_t n = 0;
  int c;
  graphscale = 7;
  tickscale=200;
  if (file == NULL) return; //could not open file
  int comment = 0;
  long l=0;
  while ((c = fgetc(file)) != EOF) {
    //if (l>35)break;
    if (c == ';')comment = 1;
    code[n++] = (char) c;
    if (c == '\n') {
      code[n++] = 0;
      l++;
      printf("\n%s", code);
      if (!comment)gcode_parse_char(c);
      n = 0;
      comment = 0;
      //getch();
    } else
    {
      if (!comment)gcode_parse_char(c);

    }
  }


}

void demo() {
  graphscale = 10;
  tickscale=160;
  fscale=8;
  int f = 100;
  amove(20, 10, 0, 0, 0);
  amove(10, 20, 0, 0, 0);
/*  
  amove(30, 10, -10, 0, 0);
  amove(30, 25, 0, 0, 1);
  amove(30, 25, 0, 1, 2);
  amove(30, 30, 0, 1, 3);
  amove(30, 40, -10, 1, 3);
  amove(30, 30, 0, 0, 4);
  amove(30, 80, 0, 0, 5);

/*  
  amove(f, 43, 0, 0, 3);
  amove(f, 54, 0, 0, 4);
  amove(f, 65, 0, 0, 5);
  amove(f, 77, 0, 0, 6);
  amove(f, 100, 0, 0, 0);
  amove(f, 110, 0, 0, 0);
  amove(f, 50, 0, 0, 0);
  amove(f, 53, 0, 0, 0);
  amove(50, 70, 0, 0, 0);
  amove(f, 80, 0, 0, 0);
  amove(f, 90, 0, 0, 0);
*/
  int i;
  /*
    for (i = 0; i < 5; i++) {
      addmove(f, i * 10, 50, 0, 0);
    }
    addmove(f, 60, 55, 0, 0);
    for (i = 5; i > 0; i--) {
      addmove(f, i * 10, 60, 0, 0);
    }
  addmove(f, 0, 65, 0, 0);
  for (i = 0; i < 36; i++) {
    addmove(f, sin(i * 44.0 / 7 / 36) * 140,  -cos(i * 44.0 / 7 / 36) * 40, 0, 1);
  }
  */


}



#endif

