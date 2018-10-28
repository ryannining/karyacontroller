#include "motion.h"
#include "timer.h"
#include "common.h"
#include <stdint.h>

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
    zprintf(PSTR("Graphics error: %s\n"), grapherrormsg(errorcode));
    zprintf(PSTR("Press any key to halt:"));

    getch();
    exit(1);               /* terminate with an error code */
  }
  zprintf(PSTR("Simple Motion Control with Acceleration, and lookahead planner\n"));
  zprintf(PSTR("By ryannining@gmail.com\n"));
  initmotion();
  struct palettetype pal;
  //getpalette(&pal);
  for (int i = 0; i < 256; i++) {
    //setrgbpalette(i,i,i,i);
  }
  setcolor(1);
  graphscale = 25;
  line(0, 400, 600, 400);
  line(0, 400 - 50 * fscale, 600, 400 - 50 * fscale);
  line(0, 400 - 100 * fscale, 600, 400 - 100 * fscale);

  //int8_t z=100;
  float v = 10.1234;
  zprintf (PSTR("F %f D %d\n"), ff(v), (int32_t)200);
  demofile();
  demo();
  zprintf (PSTR("WAIT\n"));
  waitbufferempty();
  zprintf (PSTR("Time:%f\n"), ff(tick / timescale));
  zprintf(PSTR("Total step:%d\n"),fi(mm_ctr));
  getch();
}
void demofile() {
  //#define fn "d:/git/hipopotamo.gcode"
  //#define fn "d:/3d/5050.gcode"
  //#define fn "d:/3d/font.gcode"
  //#define fn "d:/git/bowdenlock.gcode"
  //#define fn "d:/3d/fish_fossilz.gcode"
  //#define fn "d:/3d/cube20.gcode"
  //#define fn "d:/3d/bowdenlock.gcode"
  //#define fn "d:/3d/gecko.gcode"
  //#define fn "d:/3d/foam.gcode"
  //#define fn "gcode/gecko.gcode"
  //#define fn "gcode/fish_fossilz.gcode"
  //#define fn "gcode/kotak.gcode"
  //#define fn "d:/3d/box1cm.gcode"
  //#define fn "gcode/bulet.gcode"
  #define fn "gcode/naga.ngc"
  //#define fn "d:/3d/testarc.gcode"
  FILE *file = fopen(fn, "r");
  char code[100];
  size_t n = 0;
  int c;
  graphscale = 1;
  tickscale = 150;
  fscale=20;
  if (file == NULL) return; //could not open file
  int comment = 0;
  long l = 0;
  while ((c = fgetc(file)) != EOF) {
    //if (l > 35)break;
    if (c == ';')comment = 1;
    code[n++] = (char) c;
    if (c == '\n') {
      code[n++] = 0;
      l++;
      printf("\n%s", code);
      if ((l > 1) && !comment)gcode_parse_char(c);
      n = 0;
      comment = 0;
      //getch();
    } else
    {
      if ((l > 1) && !comment)gcode_parse_char(c);

    }
  }


}

void demo() {
  graphscale = 5;
  tickscale = 360;
  fscale = 60;
  int f = 100;
  ///*
  for(int x=1;x<36;x++){
 //   amove(20, sin(x/5.7)*5, cos(x/5.7)*10, 0, 0);
  }
  for(int x=1;x<360;x++){
    amove(30, +sin(x/5.70)*(2+x/40.0), cos(x/5.70)*(2+x/60.0), sin(x/5.70)*1, 0*x/10.0);
  }
   //*/
  /*amove(50, 50, 0, 0, 1,1);
  amove(50, 100, 0, 1, 1,1);
  amove(50, 30, 0, 1, 1,1);
*/
//  amove(80, 30, 0, 0, 0);
//  amove(80, 30, 30, 0, 0);
//  amove(80, 30, 50, 0, 0);
  //amove(40, 50, 20, 0, 0);
  //amove(40, 60, 10, 0, 0);
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

