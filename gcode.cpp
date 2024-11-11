

#include "common.h"
#include "gcode.h"
#include "timer.h"
#include "temp.h"
#include "eprom.h"
#include "gcodesave.h"

#if defined(ESP32) || defined(ESP8266)

// ==========================
// #ifdef ESP8266
// #include <ESP8266WiFi.h>
// #include <SPIFFS.h>   // Include the LittleFS library
// // ==========================
// #elif ESP32
// #include <WiFi.h>
// #include <SPIFFS.h>
// // ==========================
// #endif
#include <WiFiClient.h>
extern IPAddress ip ;

#endif

int32_t linecount, lineprocess;

#define MLOOP


#include<stdint.h>
#include<Arduino.h>
/// crude crc macro
#define crc(a, b)		(a ^ b)
decfloat read_digit;
int sdcardok = 0;
int waitforline = 0;
// g_str can hold upto 1024 char on ARM and 128 on avr
// to receive laser raster bitmap
char g_str[g_str_len];

uint8_t okxyz;
int g_str_c = 0;

int g_str_l = 0;
int cutpause=10000;
GCODE_COMMAND next_target;
uint16_t last_field = 0;
/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
extern void wifi_loop();


bool waitexecute=false;

int reset_command() {
  // reset variables
  //zprintf(PSTR("reset_command\n"));
  //if (ok)zprintf(PSTR("ok\n")); // response quick !!

  next_target.seen_X = next_target.seen_Y = next_target.seen_Z = \
                       next_target.seen_E = next_target.seen_F = next_target.seen_S = \
                           next_target.seen_P = next_target.seen_T = \
                               next_target.seen_G = next_target.seen_M = \
                                   next_target.read_string = g_str_c  = 0;
  MLOOP
#ifdef ARC_SUPPORT
  next_target.seen_R = next_target.seen_I = next_target.seen_J = 0;
#endif
  last_field = 0;
  read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;

  if (next_target.option_all_relative) {
    next_target.target.axis[nX] = next_target.target.axis[nY] = next_target.target.axis[nZ] = 0;
  }
  if (next_target.option_all_relative || next_target.option_e_relative) {
    next_target.target.axis[nE] = 0;
  }
  return 2;
}
int tryexecute(){
  
  if (!waitexecute)return 0;
	if (nextbuff(head) == tail) { // pending operation if buffer is full
		return 0;
	}
  if ((head!=tail) || (cmhead !=cmtail)){ // still have moves, some gcodes need to wait until trully empty
//    if (!lasermode) {
//      if (next_target.seen_M && next_target.M==3) return 0;// M3 need buffer to be empty for cnc
//    }

    if (next_target.seen_M && next_target.M==3) {
      if (!lasermode)return 0;// M3 need buffer to be empty for cnc
      if (next_target.seen_P && next_target.P>=0) return 0;
    }

    if (next_target.seen_M && next_target.G==109) return 0;// G92 need buffer to be empty
    if (next_target.seen_G && next_target.G==92) return 0;// G92 need buffer to be empty
  }
    MLOOP

    okxyz = next_target.seen_X || next_target.seen_Y || next_target.seen_Z || next_target.seen_E || next_target.seen_F;

    uint8_t ok = next_target.seen_G || next_target.seen_M || next_target.seen_T || okxyz;
	
    if (ok){
      process_gcode_command();
      //zprintf(PSTR("ok\n")); // response quick !!
      reset_command();
    }
    waitexecute=false;
    return 1;
}
void update_pos(void) {
  next_target.target.axis[nX] = cx1;
  next_target.target.axis[nY] = cy1;
  next_target.target.axis[nZ] = ocz1;
  next_target.target.axis[nE] = ce01;
}



// implement minimalis code to match teacup

float lastE;
int overridetemp = 0;
void printposition()
{
  //zprintf(PSTR("X:%f Y:%f Z:%f E:%f\n"),      ff(info_x), ff(info_y),       ff(info_z), ff(ce01));

}
void printbufflen()
{
  //zprintf(PSTR("Buf:%d\n"), fi(bufflen));

}
void pausemachine()
{
  PAUSE = !PAUSE;
  if (PAUSE)zprintf(PSTR("Pause\n"));
  else zprintf(PSTR("Resume\n"));
}

#include "ir_oled.h"

bool stopping=false;
void stopmachine(){
  stopping=true;
  laserOn=0;
}
void stopmachine2() {
  // soft stop
  //PAUSE = 1;
  //waitbufferempty(false);
  extern void doHardStop();
  doHardStop();
  enduncompress(true);
  PAUSE=0;
  stopping=false;
  //set_tool(0);
  extern int32_t pwm_val;
  if (lasermode) pwm_val=0;
}

//#define queue_wait() needbuffer()
#define queue_wait() 

void delay_ms(uint32_t d)
{

  /*while (d) {
    d--;
    somedelay(1000);
  }*/
  uint32_t start_time = micros();
  while (micros() - start_time < d) {
    // Yield to allow other tasks to run
    yield();
  }
}

void temp_wait(void)
{

}

int lastB = 0;

void str_wait()
{

  //uint32_t c = millis();
  while (lastB > 5) {
    domotionloop
    //delayMicroseconds(10);
  }
}
//int32_t mvc = 0;

int testlaserdur=3000;
void testLaser(void) {

  for (int j = testlaserdur; j--;) {
    TOOL1(TOOLON)
    domotionloop
  }

  // after some delay turn off laser
  TOOL1(!TOOLON);
}

static void enqueue(GCODE_COMMAND *) __attribute__ ((always_inline));

float F0 = 5000;
float F1 = 2000;
uint8_t S1 = 255;
inline void enqueue(GCODE_COMMAND *t, int g0 = 1)
{
  if (t->seen_F) {
    if (g0)F0 = t->target.F; else F1 = t->target.F;
  }

  amove(g0 ? F0 : F1, t->seen_X ? t->target.axis[nX] : cx1
        , t->seen_Y ? t->target.axis[nY] : cy1
        , t->seen_Z ? t->target.axis[nZ] : ocz1
        , t->seen_E ? t->target.axis[nE] : ce01
        , g0, 0);
}
#ifdef ARC_SUPPORT

inline void enqueuearc(GCODE_COMMAND *t, float I, float J, int cw)
{

  if (t->seen_F) {
    F1 = t->target.F;
  }
  draw_arc(F1, t->seen_X ? t->target.axis[nX] : cx1
           , t->seen_Y ? t->target.axis[nY] : cy1
           , t->seen_Z ? t->target.axis[nZ] : ocz1
           , t->seen_E ? t->target.axis[nE] : ce01
           , I, J, cw);
}
#endif
int lastG = 0;
int probex1, probey1;
int probemode = 0;

int lastS = 0;
void zeroall(){
  cx1 = next_target.target.axis[nX] =
                  cy1 = next_target.target.axis[nY] =
                          cz1 = ocz1 = next_target.target.axis[nZ] =
                                         ce01 = next_target.target.axis[nE] = 0;
}
void process_gcode_command()
{
  uint32_t	backup_f;

  // convert relative to absolute
  if (next_target.option_all_relative) {
    next_target.target.axis[nX] += cx1;//startpoint.axis[nX];
    next_target.target.axis[nY] += cy1;//startpoint.axis[nY];
    next_target.target.axis[nZ] += ocz1;//startpoint.axis[nZ];
    next_target.target.axis[nE] += ce01;//startpoint.axis[nZ];
  }

  // E relative movement.
  // Matches Sprinter's behaviour as of March 2012.
  if (next_target.option_all_relative || next_target.option_e_relative)
    next_target.target.e_relative = 1;
  else
    next_target.target.e_relative = 0;

  if (next_target.seen_T) {
    //? --- T: Select Tool ---
    //?
    //? Example: T1
    //?
    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

    //next_tool = next_target.T;
  }
  // check if buffer is near full
  /*
    int bl=bufflen();
    float spd=1;
    if (bl>NUMBUFFER/2) {
      spd=(float)NUMBUFFER/(bl*4+2);
    }
  */
  if (!next_target.seen_M) {
    if (!next_target.seen_G) {
      if (lastG > 1)return;
      if (!okxyz)return;
      next_target.G = lastG;
    }

    lastG = next_target.G;
    uint8_t axisSelected = 0;
    //zprintf(PSTR("Gcode G%d \n"),fi(next_target.G));
    switch (next_target.G) {
      case 0:
        //? G0: Rapid Linear Motion
        //?
        //? Example: G0 X12
        //?
        //? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
        //?
        //laserOn = 0;
        //constantlaserVal = 0;
        enqueue(&next_target, 1);
        break;

      case 1:
        //? --- G1: Linear Motion at Feed Rate ---
        //?
        //? Example: G1 X90.6 Y13.8 E22.4
        //?
        //? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
        //?
        //next_target.target.axis[nE]=0;
        // auto retraction change

        // thread S parameter as value of the laser, in 3D printer, donot use S in G1 !!
        if (next_target.seen_S) {
          S1 = next_target.S;
        }
        laserOn = S1 > 0;
        //zprintf(PSTR("int %d\n"), fi(S1));
        constantlaserVal = S1;
        enqueue(&next_target, 0);
        if (laserOn) {
        }
        break;

      //	G2 - Arc Clockwise
      //	G3 - Arc anti Clockwise
      case 2:
      case 3:
#ifdef ARC_SUPPORT
        temp_wait();

        if (!next_target.seen_I) next_target.I = 0;
        if (!next_target.seen_J) next_target.J = 0;
        //if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
#define isCW (next_target.G == 2)
        if (next_target.seen_R) {
          float r = next_target.R;
          float x = next_target.seen_X ? next_target.target.axis[nX] - cx1 : 0;
          float y = next_target.seen_Y ? next_target.target.axis[nY] - cy1 : 0;

          float h_x2_div_d = 4 * r * r - x * x - y * y;
          if (h_x2_div_d < 0) {
            break;
            //FAIL(STATUS_ARC_RADIUS_ERROR);
            //return (gc.status_code);
          }
          // Finish computing h_x2_div_d.
          h_x2_div_d = -sqrt(h_x2_div_d) / hypot(x, y); // == -(h * 2 / d)
          // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
          if (!isCW) {
            h_x2_div_d = -h_x2_div_d;
          }
          if (r < 0) {
            h_x2_div_d = -h_x2_div_d;
            r = -r; // Finished with r. Set to positive for mc_arc
          }
          // Complete the operation by calculating the actual center of the arc
          next_target.I = 0.5 * (x - (y * h_x2_div_d));
          next_target.J = 0.5 * (y + (x * h_x2_div_d));
        }

        enqueuearc(&next_target, next_target.I, next_target.J, isCW);
#endif
        break;

      case 4:
 
        break;

      case 7:
        // WE NEED TO REIMPLEMENT THE G7 COMMAND
        break;
      case 28:
        if (lasermode==2) {
          // Gcode G28 Z40 mean
          // Probe at this point then ready to cut
          // Z 40 mean distance between float Z idle and hit the limit switch is 40mm
          if (next_target.seen_Z) {
            //MESHLEVELING = 0;
            //addmove(4000, next_target.target.axis[nX], next_target.target.axis[nY], ocz1, ce01, 1, 0);
            extern float pointProbing(float floatdis);
            float zz = pointProbing(next_target.target.axis[nZ]);
            //zprintf(PSTR("%f\n"), ff(zz));
          }
        } else {
          homing();
          update_pos();
          printposition();
        }
        break;

      case 90:
        //? --- G90: Set to Absolute Positioning ---
        //?
        //? Example: G90
        //?
        //? All coordinates from now on are absolute relative to the origin
        //? of the machine. This is the RepRap default.
        //?
        //? If you ever want to switch back and forth between relative and
        //? absolute movement keep in mind, X, Y and Z follow the machine's
        //? coordinate system while E doesn't change it's position in the
        //? coordinate system on relative movements.
        //?

        // No wait_queue() needed.
        next_target.option_all_relative = 0;
        break;

      case 91:
        //? --- G91: Set to Relative Positioning ---
        //?
        //? Example: G91
        //?
        //? All coordinates from now on are relative to the last position.
        //?

        // No wait_queue() needed.
        next_target.option_all_relative = 1;
        break;

      case 92:
        //? --- G92: Set Position ---
        //?
        //? Example: G92 X10 E90
        //?
        //? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.
        //?
        //waitbufferempty();
        queue_wait();
        float lx;
        lx = cx1;
        float ly;
        ly = cy1;
        if (next_target.seen_X) {
          cx1 = next_target.target.axis[nX];
          axisSelected = 1;
        };
        if (next_target.seen_Y) {
          cy1 = next_target.target.axis[nY];
          axisSelected = 1;
        };
        if (next_target.seen_Z) {
          cz1 = ocz1 = next_target.target.axis[nZ];
          axisSelected = 1;
        };
        if (next_target.seen_E) {
          lastE = ce01 = next_target.target.axis[nE];
          axisSelected = 1;
        };

        if (axisSelected == 0) {
          zeroall();
        }
        init_pos();
        /*if (MESHLEVELING) {
          lx -= cx1;
          ly -= cy1;
          // normalize the data
          for (int j = 0; j < XCount; j++) {
            ZValues[j + 1][0] -= lx;
          }
          for (int j = 0; j < YCount; j++) {
            ZValues[0][j + 1] -= ly;
          }
          // activate leveling
          }*/

        break;

      // unknown gcode: spit an error
      default:
        //zprintf(PSTR("E:G%d\nok\n"), next_target.G);
        return;
    }
  } else if (next_target.seen_M) {
    //uint8_t i;
    //zprintf(PSTR("Gcode M%d \n"),fi(next_target.M));

    switch (next_target.M) {
      case 0:
      //? --- M0: machine stop ---
      //?
      //? Example: M0
      //?
      //? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
      //? Unimplemented, especially the restart after the stop. Fall trough to M2.
      //?

      case 5:
        next_target.seen_S = true;
        next_target.S = 0;
      case 3:
        if (!next_target.seen_S)next_target.S = 255;
        //if (next_target.S > 4000)next_target.S = next_target.S / 100; // convert 33K RPM max to 255
        //if (fi(next_target.S) == lastS)break;
        //lastS = fi(next_target.S);
        if (!next_target.seen_P) next_target.P = 0;
        if (lasermode==2 && next_target.S>10)next_target.S=255;
        set_tool(next_target.S);
        if (next_target.P >= 10000) {
          extern void dopause(int tm=0,bool stopping=false);
          next_target.P=0;
          next_target.seen_P=0;
          dopause(cutpause,false);
        }

        // if no S defined then full power
        S1 = next_target.S;


        if (next_target.seen_P) {
          //waitbufferempty();
          //zprintf(PSTR("PULSE LASER\n"));


          TOOL1(TOOLON)
          
          int w1=millis();
          while (millis()-w1<next_target.P) {
            //delay(10);
            domotionloop
          }

          // after some delay turn off laser
          //TOOL1(!TOOLON);

        }
        break;
        /*      case 101:
                //? --- M101: extruder on ---
                //?
                //? Undocumented.
                temp_wait();
                // enable the laser or spindle
                break;

              // M5/M103- extruder off
              case 5:
              case 103:
                //? --- M103: extruder off ---
                //?
                //? Undocumented.
                // disable laser/spindle
                break;
        */


      
    } // switch (next_target.M)
  } // else if (next_target.seen_M)
} // process_gcode_command()

void init_gcode()
{
  next_target.target.F = 50;
  next_target.option_all_relative = 0;
}
