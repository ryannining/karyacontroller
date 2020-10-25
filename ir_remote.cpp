
// please do not use D8 on esp8266 / wemos. IR not work there.

#include "common.h"
#include "config_pins.h"

#ifdef IR_KEY
#include "motion.h"
#include <math.h>
#include "timer.h"
#include "temp.h"
#include <stdint.h>

#include "gcodesave.h"
#include "IRLremote.h"
#include "ir_remote.h"
#include "ir_oled.h"
CNec IRLremote;
bool IR_ok;
int wait_job = 0;
int wait_spindle = 0;
String jobnum;
void IR_setup() {
  IR_ok = IRLremote.begin(IR_KEY);
  //if (IR_ok) zprintf(PSTR("IR Key OK.\n"));
}
void IR_end() {
  IRLremote.end(IR_KEY);
}

extern void wifi_push(char c); // we use WIFI GCODE Command buffer to inject gcode
const char *gc92 = "G92\n";
int test1 = 0;
extern int tmul;
void IR_loop() {

  if (!IR_ok)return;
  if (IRLremote.available())
  {
    auto data = IRLremote.read();
    int ok = ir_oled_loop(data.command);
    if (ok)return; // the key is consumed by the display
    //zprintf(PSTR("Key:%d \n"),fi(data.command));
    extern int8_t RUNNING;
    extern int uncompress;
    // if not uncompress/running internal Gcode, we can JOG
    if (!uncompress) {
      // JOG

      float x, y, z;
      x = 0;
      y = 0;
      z = 0;

      // jog on arrow key = small incremental
      switch (data.command) {
        case IRK_UP: // UP key
          y = -1; break;
        case IRK_LF: // LEFT key
          x = -1; break;
        case IRK_RG: // RIGHT key
          x = 1;  break;
        case IRK_DN: // DOWN key
          y = 1;  break;
      }
      // if not waiting job id (* followed by number), we can use number as fast jog and Z jog
      if (!(wait_job || wait_spindle))
        switch (data.command) {
          case IRK_1: z = 1; break;
          case IRK_7: z = -1; break;
          case IRK_3: z = 0.5; break;
          case IRK_9: z = -0.5; break;
          case IRK_5: tmul = tmul==1?5:1; break;


          case IRK_2: y = -10; break;
          case IRK_4: x = -10; break;
          case IRK_6: x = 10; break;
          case IRK_8: y = 10; break;
          case IRK_OK: // sethome pos
            for (int i = 0; i < strlen(gc92); i++) wifi_push(gc92[i]);
            break;
          case IRK_0: // 0 = home return
            addmove(100, 0, 0, 10, 0, 1, 1);
            addmove(100, 0, 0, 10, 0, 1, 0);
            addmove(100, 0, 0, 0, 0, 1, 0);
            return;
        }
		
      addmove(100, x*tmul, y*tmul, z, 0, 1, 1);
      switch (data.command) {
        case IRK_X: //*
          //wait_job = 1; wait_spindle = 0; jobnum = "/"; 

          break;
        case IRK_H: //#
          // run job
          if (wait_job) {
            jobnum += ".gcode";
            beginuncompress(jobnum);
            wait_job = 0;
            //zprintf(PSTR("Job : %d \n"), fi(jobnum));
          } else {
            wait_spindle = 1;
          }

          break;
      }

    } else {
      // special when uncompress/runnning

      switch (data.command) {
        /*case IRK_OK: // pause ====== we use OK to enter menu, so we should stop job in the menu also
        	extern int ispause;
        	ispause=ispause ? 0:1;
        	return;
        */
        case IRK_X: //* stop
          /*enduncompress();
          extern void stopmachine();
          stopmachine();*/
          extern int ispause;
          ispause=ispause ? 0:1;
          return;
        case IRK_H:
          wait_spindle = 1;	break;
        // baby step is change the XYZ small value while its running job
        case IRK_1: babystep[2] += 250; break;
        case IRK_7: babystep[2] -= 250; break;
        case IRK_3: babystep[2] += 100; break;
        case IRK_9: babystep[2] += -100; break;

        case IRK_2: babystep[1] -= 250; break;
        case IRK_8: babystep[1] += 250; break;
        case IRK_4: babystep[0] -= 250; break;
        case IRK_6: babystep[0] += 250; break;

        case IRK_UP: babystep[1] -= 100; break;
        case IRK_DN: babystep[1] += 100; break;
        case IRK_LF: babystep[0] -= 100; break;
        case IRK_RG: babystep[0] += 100; break;
      }

    }
    int num = -1;
    if (wait_job || wait_spindle  ) {
      switch (data.command) {
        case IRK_1: num = 1; break;
        case IRK_2: num = 2; break;
        case IRK_3: num = 3; break;
        case IRK_4: num = 4; break;
        case IRK_5: num = 5; break;
        case IRK_6: num = 6; break;
        case IRK_7: num = 7; break;
        case IRK_8: num = 8; break;
        case IRK_9: num = 9; break;
        case IRK_0: num = 0; break;
      }
    }
    if (num >= 0) {
      if (wait_spindle) {
        set_pwm(num * 28.3);
        wait_spindle = 0;
        return;
      }
      else if (wait_job) {
        jobnum += num;
        wait_job++;
      }
    }

  } else ir_oled_loop(0);
}

#else
void IR_setup() {}
void IR_end() {}
void  IR_loop() {}
#endif

