

#include "common.h"
#include "gcode.h"
#include "timer.h"
#include "temp.h"
#include "eprom.h"
#include "gcodesave.h"

#if defined(ESP32) || defined(ESP8266)

// ==========================
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <FS.h>   // Include the SPIFFS library
// ==========================
#elif ESP32
#include <WiFi.h>
#include "SPIFFS.h"
// ==========================
#endif


//#include <WiFiClient.h>
File fme;

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

GCODE_COMMAND next_target;
uint16_t last_field = 0;
/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
extern void wifi_loop();

static float decfloat_to_float(void)
{
  float r = read_digit.mantissa;
  uint8_t	e = read_digit.exponent;
  /*  uint32_t powers=1;
    for (e=1; e<read_digit.exponent;e++) powers*=10;
    // e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
  */
  if (e) r = (r /*+ powers[e-1] / 2*/) / POWERS(e - 1);
  //  if (e) r = (r /*+ powers[e-1] / 2*/) * POWERS(e - 1);
  MLOOP
  return read_digit.sign ? -r : r;
}
void changefilament(float l)
{
#ifdef CHANGEFILAMENT
  waitbufferempty();
  float backupE = ce01;
  float backupX = cx1;
  float backupY = cy1;
  float backupZ = ocz1;

  addmove(50, 0, 0, 0, -2, 0, 1); // retract
  addmove(50, 0, 0, 30, 0, 0, 1); // move up
  addmove(50, 0, 0, 0, -l, 0, 1); // unload filament
  waitbufferempty();
  checkendstop = 1;
  //zprintf(PSTR("change filemant, then push endstop\n"));
  while (1) {
    docheckendstop(0);
    if (endstopstatus < 0) break;
    domotionloop

  }
  checkendstop = 0;
  addmove(5, 0, 0, 0, l + 10, 0, 1); // load filament
  addmove(50, 0, 0, -30, 0, 0, 1);
  waitbufferempty();
  ce01 = backupE;
  cx1 = backupX;
  cy1 = backupY;
  ocz1 = backupZ;
#endif
}
bool waitexecute=false;

int reset_command() {
  // reset variables
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
    if (!lasermode) {
      if (next_target.seen_M && next_target.M==3) return 0;// M3 need buffer to be empty for cnc
    }
    if (next_target.seen_M && next_target.G==109) return 0;// G92 need buffer to be empty
    if (next_target.seen_G && next_target.G==92) return 0;// G92 need buffer to be empty
  }
    MLOOP

    okxyz = next_target.seen_X || next_target.seen_Y || next_target.seen_Z || next_target.seen_E || next_target.seen_F;

    uint8_t ok = next_target.seen_G || next_target.seen_M || next_target.seen_T || okxyz;
	
    if (ok){
      process_gcode_command();
      zprintf(PSTR("ok\n")); // response quick !!
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
uint8_t gcode_parse_char(uint8_t c)
{
  uint8_t checksum_char = c;
  //serialwr(c);
  // uppercase
  if (c >= 'a' && c <= 'z' && !next_target.read_string)
    c &= ~32;

  // An asterisk is a quasi-EOL and always ends all fields.
  if (c == '*') {
    //next_target.read_string = 0;
  }

  // Skip comments and strings.
  if (
    next_target.read_string == 0
  ) {
    // Check if the field has ended. Either by a new field, space or EOL.
    if (last_field && (c < '0' || c > '9') && c != '.') {
      switch (last_field) {
        case 'G':
          next_target.G = read_digit.mantissa;
          break;
        case 'M':
          next_target.M = read_digit.mantissa;
          if (next_target.M == 117) next_target.read_string = 1;

          break;
        case 'X':

          next_target.target.axis[nX] = decfloat_to_float();
          break;
        case 'Y':
          next_target.target.axis[nY] = decfloat_to_float();
          break;
        case 'Z':
          next_target.target.axis[nZ] = decfloat_to_float();
          break;
#ifdef ARC_SUPPORT
        case 'I':
          next_target.I = decfloat_to_float();
          break;
        case 'J':
          next_target.J = decfloat_to_float();
          break;
        case 'R':
          next_target.R = decfloat_to_float();
          break;
#endif
        case 'E':
          next_target.target.axis[nE] = decfloat_to_float();
          break;
        case 'F':
          // just use raw integer, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
          next_target.target.F = decfloat_to_float() / 60;
          MLOOP
          break;
        case 'S':
          next_target.S = decfloat_to_float();
          break;
        case 'P':
          next_target.P = decfloat_to_float();
          break;
        case '*':
          //next_target.checksum_read = decfloat_to_float();
          break;
        case 'T':
          //next_target.T = read_digit.mantissa;
          break;
        case 'N':
          //next_target.N = decfloat_to_float();
          break;

      }
    }

    // new field?
    if ((c >= 'A' && c <= 'Z') || c == '*') {
      last_field = c;
      read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
    }

    // process character
    // Can't do ranges in switch..case, so process actual digits here.
    // Do it early, as there are many more digits than characters expected.
    if (c >= '0' && c <= '9') {
      if (read_digit.exponent < DECFLOAT_EXP_MAX + 1) {
        // this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
        read_digit.mantissa = (read_digit.mantissa * 10) + (c - '0');
        if (read_digit.exponent)
          read_digit.exponent++;
      }
    } else {
      switch (c) {
        // Each currently known command is either G or M, so preserve
        // previous G/M unless a new one has appeared.
        // FIXME: same for T command
        case 'G':
          next_target.seen_G = 1;
          //next_target.seen_M = 0;
          //next_target.M = 0;
          break;
        case 'M':
          next_target.seen_M = 1;
          //next_target.seen_G = 0;
          //next_target.G = 0;
          break;
#ifdef ARC_SUPPORT
        case 'I':
          next_target.seen_I = 1;
          break;
        case 'J':
          next_target.seen_J = 1;
          break;
        case 'R':
          next_target.seen_R = 1;
          break;
#endif
        case 'X':
          next_target.seen_X = 1;
          break;
        case 'Y':
          next_target.seen_Y = 1;
          break;
        case 'Z':
          next_target.seen_Z = 1;
          break;
        case 'E':
          next_target.seen_E = 1;
          break;
        case 'F':
          next_target.seen_F = 1;
          break;
        case 'S':
          next_target.seen_S = 1;
          break;
        case 'P':
          next_target.seen_P = 1;
          break;

        case 'T':
          next_target.seen_T = 1;
          break;
        case 'N':
          //next_target.seen_N = 1;
          break;
        case '*':
          //next_target.seen_checksum = 0;//1;
          break;
        // comments
        case '[':
          next_target.read_string = 1;  // Reset by ')' or EOL
          g_str_l = 0;
          if (next_target.seen_P && next_target.seen_G) g_str_c = next_target.P; else g_str_c = 0;
          if (next_target.G == 7)str_wait();
          break;

        case ';':
          next_target.read_string = 1;   // Reset by EOL.
          break;
        // now for some numeracy
        case '-':
          read_digit.sign = 1;
          // force sign to be at start of number, so 1-2 = -2 instead of -12
          read_digit.exponent = 0;
          read_digit.mantissa = 0;
          break;
        case '.':
          if (read_digit.exponent == 0)
            read_digit.exponent = 1;
          break;
#ifdef	DEBUG
        case ' ':
        case '\t':
        case 10:
        case 13:
          // ignore
          break;
#endif

        default:
#ifdef	DEBUG
          // invalid
          //zprintf(PSTR("?%d\n"), fi(c));
#endif
          break;
      }
    }
  } //else if ( next_target.seen_parens_comment == 1 && c == ')')
  else {
    // store string in g_str  from gcode example M206 P450 [ryan widi]
    if (c == ']' || c == 10 || c == 13) {
      g_str[g_str_c] = 0;
      next_target.read_string = 0;
    } else {
      if (g_str_c < g_str_len - 1) {
        g_str[g_str_c] = c;
        g_str_c++;
        //g_str_c=g_str_c&8191;
        g_str_l++;
      }
    }
    //next_target.seen_parens_comment = 0; // recognize stuff after a (comment)
  }


  // end of line
  if ((c == 10) || (c == 13)) {
	  waitexecute=true;
  }

  return 0;
}


// implement minimalis code to match teacup

float lastE;
int overridetemp = 0;
void printposition()
{
  zprintf(PSTR("X:%f Y:%f Z:%f E:%f\n"),
          ff(info_x), ff(info_y),
          ff(info_z), ff(ce01));

}
void printbufflen()
{
  zprintf(PSTR("Buf:%d\n"), fi(bufflen));

}
void pausemachine()
{
  PAUSE = !PAUSE;
  if (PAUSE)zprintf(PSTR("Pause\n"));
  else zprintf(PSTR("Resume\n"));
}
#ifdef IR_OLED_MENU
#include "ir_oled.h"
#endif
bool stopping=false;
void stopmachine(){
  stopping=true;
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
if (lasermode)
  TOOL1(!TOOLON);

}

//#define queue_wait() needbuffer()
#define queue_wait() 

void delay_ms(uint32_t d)
{

  while (d) {
    d--;
    somedelay(1000);
  }


}
void temp_wait(void)
{
#ifdef heater_pin
  wait_for_temp = 1;
  uint32_t c = millis();
  while (wait_for_temp && !temp_achieved()) {
    domotionloop
    wifi_loop();
    //report each second
    if (millis() - c > 1000) {
      c = millis();
      zprintf(PSTR("T:%f\n"), ff(Input));
      //zprintf(PSTR("Heating\n"));
    }
  }
  wait_for_temp = 0;
#endif
}

int lastB = 0;

void str_wait()
{

  //uint32_t c = millis();
  while (lastB > 5) {
    domotionloop
    MEMORY_BARRIER()
    //delayMicroseconds(10);
  }
}
//int32_t mvc = 0;

typedef struct {
  float pX, pY;
  uint8_t bit;
} tlaserdata;

bool collectLaser = false;
bool runLaser = false;
int laseridx = 0;
int laseridxrun = 0;
tlaserdata laserdata[200];
tlaserdata laserdatarun[200];
void runlasernow() {
  // if still running, lets wait
  while (runLaser) {
    motionloop();
  }
  laseridxrun = 0;
  runLaser = 1;
  memcpy(&laserdatarun, &laserdata, sizeof(laserdata));
}

void addlaserxy(float x, float y, uint8_t bit)
{
  laserdata[laseridx].pX = x;
  laserdata[laseridx].pY = y;
  laserdata[laseridx].bit = bit;


  laseridx++;
  if (laseridx > 199) {
    runlasernow();
  }
}

void testLaser(void) {

  for (int j = 3000; j--;) {
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

void printmeshleveling() {
#ifdef MESHLEVEL
  for (int j = 0; j <= YCount; j++) {
    for (int i = 0; i <= XCount; i++) {
      if (i)zprintf(PSTR("\t"));
      if (j == 0 && i > 0)zprintf(PSTR("%d"), fi(i));
      else if (i == 0 && j > 0)zprintf(PSTR("%d"), fi(j));
      else if (i == 0 && j == 0)zprintf(PSTR("*"));
      else zprintf(PSTR("%d"), fi(ZValues[i][j]));
    }
    zprintf(PSTR("\n"));
  }
  zprintf(PSTR("\n\n"));
#endif
}
void loadmeshleveling() {
#ifdef MESHLEVEL
#if defined(ESP8266) || defined(ESP32)
#define path "/mesh.dat"
  if (SPIFFS.exists(path)) {
    fme = SPIFFS.open(path, "r");
    fme.read((uint8_t *)&XCount, sizeof XCount);
    fme.read((uint8_t *)&YCount, sizeof YCount);
    fme.read((uint8_t *) & (ZValues[0][0]), sizeof ZValues);
    zprintf(PSTR("%d bytes\nMESH\n"), fi(fme.size()));
    zprintf(PSTR("%dx%d\n"), fi(XCount), fi(YCount));
    fme.close();
    /*for (int j = 1; j <= XCount; j++) {
      for (int i = 1; i <= YCount; i++) {
        zprintf(PSTR("%d,"),fi(ZValues[j][i]));
      }
      zprintf(PSTR("\n"));
      }
      zprintf(PSTR("\n"));
    */
  } else {
    zprintf(PSTR("File not found mesh.dat\n"));
    return;
  }
#endif
  MESHLEVELING = 1;
  printmeshleveling();
#endif
}
int lastS = 0;

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
#ifdef output_enable
      case 5:
        reset_eeprom();
        reload_eeprom();
      case 6:
        cx1 = 0;
        cy1 = 0;
        ocz1 = 0;
        ce01 = 0;
        /*
          amove(1, 100, 100, 100, 0);
          amove(100, 10, 0, 0, 0);
          amove(100, 10, 10, 0, 0);
          amove(100, 0, 10, 0, 0);
          amove(100, 0, 0, 0, 0);

          amove(100, 10, 0, 0, 0);
          amove(100, 10, 10, 0, 0);
          amove(100, 0, 10, 0, 0);
          amove(100, 0, 0, 0, 0);

          amove(100, 10, 0, 0, 0);
          amove(100, 10, 10, 0, 0);
          amove(100, 0, 10, 0, 0);
          amove(100, 0, 0, 0, 0);

          amove(100, 10, 0, 0, 0);
          amove(100, 10, 10, 0, 0);
          amove(100, 0, 10, 0, 0);
          amove(100, 0, 0, 0, 0);
        */
        break;
#endif
      case 7:
        // WE NEED TO REIMPLEMENT THE G7 COMMAND
        break;
      case 28:
#ifdef PLASMA_MODE
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
#else
        homing();
        update_pos();
        printposition();
#endif
        break;
#ifdef MESHLEVEL
      case 29:
        MESHLEVELING = next_target.seen_S;
        //zprintf(PSTR("A.L "));
        if (MESHLEVELING)zprintf(PSTR("on\n")); else zprintf(PSTR("off\n"));
        break;
      // Probing
      // mesh bed probing from current position to width , height and number of data
      // G30 Snumdata Xwidth Yheight
      // G30 S4 X100 Y100
      //
      // single probe
      // G30 Xpos Ypos
      // G30 (current position)
      case 30:
        if (next_target.seen_S) {

          MESHLEVELING = 0;

          int w = next_target.S;
          probex1 = cx1;
          probey1 = cy1;
          //float probez1 = ocz1;
          // move up before probing
          //addmove(8000, probex1 , probey1 , ocz1 + 15, ce01, 0, 0);
          int ww = next_target.target.axis[nX];
          int hh = next_target.target.axis[nY];
          XCount = floor(ww / w) + 2; // 150/200 = 0 + 2 = 2
          YCount = floor(hh / w) + 2; // 50/200 = 0 + 2 = 2
          int dx = ww / (XCount - 1); // 150/1 = 150
          int dy = hh / (YCount - 1);; // 50/1 = 50
          int zmin = 10000;
          for (int j = 0; j < YCount; j++) { // 0,1
            ZValues[0][j + 1] = (probey1 + j * dy);  // [
          }
          for (int j = 0; j < XCount; j++) {
            ZValues[j + 1][0] = (probex1 + j * dx);

            for (int ii = 0; ii < YCount; ii++) {
              int i = ii;
              if (j & 1 == 1)i = YCount - 1 - ii;

              int lz, zz, good, gz;
              good = 0;
              gz = 0;
              lz = 10 * pointProbing();

              while (good < 3) {
                addmove(8000, probex1 + j * dx, probey1 + i * dy, ocz1, ce01, 0, 0);

                zz = 10 * pointProbing();
                if (abs(zz - lz) < 3) {
                  good++;
                  gz += zz;
                }
                lz = zz;
              }
              ZValues[j + 1][i + 1] = gz / good; // average good z

            }
#ifdef WIFISERVER
            wifi_loop();
#endif
          }
          zmin = ZValues[1][1];

          // normalize the data

          for (int j = 0; j <= XCount; j++) {
            for (int i = 0; i <= YCount; i++) {
              if (i && j)ZValues[j][i] -= zmin;
            }

          }

          // activate leveling
          // back to zero position and adjust
          addmove(8000, probex1 , probey1 , ocz1, ce01, 0, 0);
          //addmove(8000, probex1 , probey1 , ocz1+ZValues[1][1], ce01, 0, 0);
          waitbufferempty();
          ocz1 = 0;
          cz1 = 0;
          printposition();
          MESHLEVELING = 1;
#if defined(ESP32) || defined(ESP8266)
          fme = SPIFFS.open("/mesh.dat", "w");
          fme.write((uint8_t *)&XCount, sizeof XCount);
          fme.write((uint8_t *)&YCount, sizeof YCount);
          fme.write((uint8_t *) & (ZValues[0][0]), sizeof ZValues);
          fme.close();
#endif
          loadmeshleveling();
        } else {

          MESHLEVELING = 0;
          if (!next_target.seen_X)next_target.target.axis[nX] = cx1;
          if (!next_target.seen_Y)next_target.target.axis[nY] = cy1;
          //zprintf(PSTR("PR X=%f Y=%f :"), ff(next_target.target.axis[nX]), ff(next_target.target.axis[nY]));
          addmove(4000, next_target.target.axis[nX], next_target.target.axis[nY], ocz1, ce01, 1, 0);

          float zz = pointProbing();
          zprintf(PSTR("%f\n"), ff(zz));

        }
        break;
      // manually store probing data
      case 31:
        // load meshleveling
        loadmeshleveling();
        break;
      case 32:
        // load meshleveling
        int ii;
        ii = next_target.target.axis[nY];
        int jj;
        jj = next_target.target.axis[nX];
        ZValues[jj][ii] = next_target.target.axis[nZ];
        printmeshleveling();
        break;
      case 33:
#if defined(ESP32) || defined(ESP8266)
        int zmin;
        zmin = ZValues[1][1];

        // normalize the data

        for (int j = 0; j <= XCount; j++) {
          for (int i = 0; i <= YCount; i++) {
            if (i && j)ZValues[j][i] -= zmin;
          }

        }
        fme = SPIFFS.open("/mesh.dat", "w");
        fme.write((uint8_t *)&XCount, sizeof XCount);
        fme.write((uint8_t *)&YCount, sizeof YCount);
        fme.write((uint8_t *) & (ZValues[0][0]), sizeof ZValues);
        fme.close();
        printmeshleveling();
#endif
        break;
#endif
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
          cx1 = next_target.target.axis[nX] =
                  cy1 = next_target.target.axis[nY] =
                          cz1 = ocz1 = next_target.target.axis[nZ] =
                                         ce01 = next_target.target.axis[nE] = 0;
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

      case 2:
        // stop and clear all buffer
        stopmachine();
        break;
      case 25:
        // stop and clear all buffer
        pausemachine();
        break;

      case 84: // For compatibility with slic3rs default end G-code.
        //? --- M2: program end ---
        //?
        //? Example: M2
        //?
        //? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
        //?
        queue_wait();
        //for (i = 0; i < NUM_HEATERS; i++)temp_set(i, 0);
        power_off();

        //zprintf(PSTR("\nstop\n"));
        break;

      /*      case 6:
              //? --- M6: tool change ---
              //?
              //? Undocumented.
              //tool = next_tool;
              break;
      */
      // M3/M101- extruder on M3 S -> PWM output to heated pin
      // M4 are special, usually to make spindle counter clockwise and we dont have implementation, but we use it for laser
      // mode : constant laser burn per step
      // M4 Sxxx   0: disable 1:enable
      // if defined, M3 Sxxx, xxx will define how manu microseconds laser will on on each step.
      // xxx will limit the G1 maximum feedrate
      //

      case 5:
        next_target.seen_S = true;
        next_target.S = 0;
      case 3:
        if (!next_target.seen_S)next_target.S = 255;
        //if (next_target.S > 4000)next_target.S = next_target.S / 100; // convert 33K RPM max to 255
        //if (fi(next_target.S) == lastS)break;
        //lastS = fi(next_target.S);
        if (!next_target.seen_P) next_target.P = 0;
        #ifdef PLASMA_MODE
        if (next_target.S>10)next_target.S=255;
        #endif 
        set_tool(next_target.S);
        if (next_target.P >= 10000) {
          extern void dopause(int tm=0);
          next_target.P=0;
          next_target.seen_P=0;
          dopause(5000);
        }

        // if no S defined then full power
        S1 = next_target.S;


        if (next_target.seen_P) {
          //waitbufferempty();
          //zprintf(PSTR("PULSE LASER\n"));

          //delay(100);
          //xpinMode(tool1_pin, OUTPUT);

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
#ifdef servo_pin
      case 300:
        //waitbufferempty();
        setfan_val(255); // turn on power
        zprintf(PSTR("Servo:%d\n"), fi(next_target.S));
        servo_set(next_target.S);
        if (!next_target.seen_P)next_target.P = 1000; // 1 second wait
        // wait loop
        uint32_t mc;
        mc = millis();
        while ((millis() - mc) < next_target.P) {
          domotionloop
        }
        setfan_val(0); // turn off power
        break;
#endif
      case 104:
        set_temp(next_target.S);
#ifdef EMULATETEMP
        extern float HEATINGSCALE;
        if (next_target.seen_P) {
          HEATINGSCALE = next_target.P / 100.0;
        }
#endif
        break;
      case 105:
        zprintf(PSTR("T:%f\n"), ff(Input));
#ifdef EMULATETEMP
#ifdef temp_pin
        zprintf(PSTR("Tx:%f\n"), ff(xInput));
#endif
#endif
        //zprintf(PSTR("TS:%f\n"), ff(Setpoint));
        //zprintf(PSTR("B:%d/%d\n"), fi(bufflen),NUMBUFFER));
        break;
      case 109:
        //waitbufferempty();
        if (overridetemp)next_target.S = overridetemp;
        overridetemp = 0;
        set_temp(next_target.S + 8);
        temp_wait();
        set_temp(next_target.S);
        break;
      case 7:
      case 107:
        // set laser pwm off
#ifdef fan_pin
        setfan_val(0);
#endif
        break;
      case 106:
        // set laser pwm on
#ifdef fan_pin
        setfan_val(next_target.S);
#endif
        break;

      case 112:
        //? --- M112: Emergency Stop ---
        //?
        //? Example: M112
        //?
        //? Any moves in progress are immediately terminated, then the printer
        //? shuts down. All motors and heaters are turned off. Only way to
        //? restart is to press the reset button on the master microcontroller.
        //? See also M0.
        //?
        // stop and clear all buffer
        //RUNNING = 0;

        break;

      case 114:
        //? --- M114: Get Current Position ---
        //?
        //? Example: M114
        //?
        //? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
        //?
        //? For example, the machine returns a string such as:
        //?
        //? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
        //?
#ifdef ENFORCE_ORDER
        // wait for all moves to complete
        queue_wait();
#endif
        printposition();
        break;

      case 115:
        //        zprintf(PSTR("FIRMWARE_NAME:Repetier_1.9 FIRMWARE_URL:null PROTOCOL_VERSION:1.0 MACHINE_TYPE:teacup EXTRUDER_COUNT:1 REPETIER_PROTOCOL:\n"));
        zprintf(PSTR("FIRMWARE_NAME:Repetier_1.9\n"));

        break;


      case 119:
        //? --- M119: report endstop status ---
        //? Report the current status of the endstops configured in the
        //? firmware to the host.
        docheckendstop(1);
        //zprintf(PSTR("END:"));
        zprintf(endstopstatus < 0 ? PSTR("HI\n") : PSTR("LOW\n"));
        //zprintf(PSTR("\n"));

        break;

        // unknown mcode: spit an error
#ifdef USE_EEPROM
      case 206:
        if (next_target.seen_X)next_target.S = next_target.target.axis[nX];
        int32_t S_F;
        S_F = (next_target.S * 1000);
        int32_t S_I;
        S_I = (next_target.S);
        if (next_target.seen_P)
          switch (next_target.P) {
#define eprom_wr(id,pos,val){\
  case id:\
    eepromwrite(pos, val);\
    break;\
  }
  /*
              eprom_wr(145, EE_xhome, S_F);
              eprom_wr(149, EE_yhome, S_F);
              eprom_wr(153, EE_zhome, S_F);
              eprom_wr(0, EE_estepmm, S_F);
              eprom_wr(3, EE_xstepmm, S_F);
              eprom_wr(7, EE_ystepmm, S_F);
              eprom_wr(11, EE_zstepmm, S_F);

              eprom_wr(15, EE_max_x_feedrate, S_I);
              eprom_wr(19, EE_max_y_feedrate, S_I);
              eprom_wr(23, EE_max_z_feedrate, S_I);
              eprom_wr(27, EE_max_e_feedrate, S_I);


              eprom_wr(51, EE_accel, S_I);


              eprom_wr(177, EE_homing, S_I);
              eprom_wr(181, EE_corner, S_I);
              eprom_wr(185, EE_Lscale, S_F);

#ifdef USE_BACKLASH
              eprom_wr(80, EE_xbacklash, S_F);
              eprom_wr(84, EE_ybacklash, S_F);
              eprom_wr(88, EE_zbacklash, S_F);
              eprom_wr(92, EE_ebacklash, S_F);
#endif

              eprom_wr(165, EE_towera_ofs, S_F);
              eprom_wr(169, EE_towerb_ofs, S_F);
              eprom_wr(173, EE_towerc_ofs, S_F);

#ifdef ANALOG_THC
              eprom_wr(157, EE_thc_up, S_I);
              eprom_wr(161, EE_thc_ofs, S_I);
#endif
              eprom_wr(300, EE_retract_in, S_F);
              eprom_wr(304, EE_retract_in_f, S_F);
              eprom_wr(308, EE_retract_out, S_F);
              eprom_wr(312, EE_retract_out_f, S_F);

              eprom_wr(316, EE_pid_p, S_F);
              eprom_wr(320, EE_pid_i, S_F);
              eprom_wr(324, EE_pid_d, S_F);
              eprom_wr(328, EE_pid_bang, S_F);
              eprom_wr(340, EE_pid_HS, S_F);
#if defined(ESP32) || defined(ESP8266)
              eprom_wr(380, EE_gcode, S_I);
#endif
              eprom_wr(332, EE_ext_adv, S_F);
              eprom_wr(336, EE_un_microstep, S_I);
*/
#ifdef WIFISERVER
            case 400:
              eepromwritestring(400, g_str);
              break;
            case 450:
              eepromwritestring(450, g_str);
              break;
            case 470:
              eepromwritestring(470, g_str);
              break;
#endif
          }
        reload_eeprom();
        break;
#ifndef SAVE_RESETMOTION
      case 502:
        reset_eeprom();
#endif
      case 205:
        reload_eeprom();
#endif
      case 503:
/*
        zprintf(PSTR("EPR:3 145 %f X Home Pos\n"), ff(ax_home[0]));
        zprintf(PSTR("EPR:3 149 %f Y\n"), ff(ax_home[1]));
        zprintf(PSTR("EPR:3 153 %f Z\n"), ff(ax_home[2]));

        zprintf(PSTR("EPR:3 3 %f X step/mm\n"), ff(stepmmx[0]));
        zprintf(PSTR("EPR:3 7 %f Y\n"), ff(stepmmx[1]));
        zprintf(PSTR("EPR:3 11 %f Z\n"), ff(stepmmx[2]));
        zprintf(PSTR("EPR:3 0 %f E\n"), ff(stepmmx[3]));

        zprintf(PSTR("EPR:2 15 %d X maxF\n"), fi(maxf[0]));
        zprintf(PSTR("EPR:2 19 %d Y\n"), fi(maxf[1]));
        zprintf(PSTR("EPR:2 23 %d Z\n"), fi(maxf[2]));
        zprintf(PSTR("EPR:2 27 %d E\n"), fi(maxf[3]));


        zprintf(PSTR("EPR:3 181 %d Corner\n"), fi(xycorner));
        zprintf(PSTR("EPR:3 51 %d Acl\n"), fi(accel));

        zprintf(PSTR("EPR:3 177 %d HomeF\n"), fi(homingspeed));
        zprintf(PSTR("EPR:3 185 %f Lscale\n"), ff(Lscale));
#ifdef USE_BACKLASH
        zprintf(PSTR("EPR:3 80 %f X Backlash\n"), fi(xback[0]));
        zprintf(PSTR("EPR:3 84 %f Y\n"), fi(xback[1]));
        zprintf(PSTR("EPR:3 88 %f Z\n"), fi(xback[2]));
        zprintf(PSTR("EPR:3 92 %f E\n"), fi(xback[3]));
#endif



#ifdef ANALOG_THC
        zprintf(PSTR("EPR:3 157 %d THCRef\n"), fi(thc_up));
        zprintf(PSTR("EPR:3 161 %d THCOfs\n"), fi(thc_ofs));
#endif

        zprintf(PSTR("EPR:3 165 %f Xofs\n"), ff(axisofs[0]));
#ifdef DRIVE_XYYZ
        zprintf(PSTR("EPR:3 169 %f Y1ofs\n"), ff(axisofs[1]));
        zprintf(PSTR("EPR:3 173 %f Y2ofs\n"), ff(axisofs[2]));
#else
        zprintf(PSTR("EPR:3 169 %f Yofs\n"), ff(axisofs[1]));
        zprintf(PSTR("EPR:3 173 %f Zofs\n"), ff(axisofs[2]));
#endif

        zprintf(PSTR("EPR:3 300 %f AtRetractIn\n"), ff(retract_in));
        zprintf(PSTR("EPR:3 304 %f F\n"), ff(retract_in_f));
        zprintf(PSTR("EPR:3 308 %f Out\n"), ff(retract_out));
        zprintf(PSTR("EPR:3 312 %f F\n"), ff(retract_out_f));

#if defined(heater_pin)
        zprintf(PSTR("EPR:3 316 %f P\n"), ff(myPID.GetKp()));
        zprintf(PSTR("EPR:3 320 %f I\n"), ff(myPID.GetKi()));
        zprintf(PSTR("EPR:3 324 %f D\n"), ff(myPID.GetKd()));
#ifdef EMULATETEMP
        zprintf(PSTR("EPR:3 328 %f ET\n"), ff(tbang));
        zprintf(PSTR("EPR:3 340 %f HS\n"), ff(HEATINGSCALE));
#endif
#endif
#if  defined(RPM_COUNTER)
        extern PID RPM_PID;
        zprintf(PSTR("EPR:3 316 %f P\n"), ff(RPM_PID.GetKp()));
        zprintf(PSTR("EPR:3 320 %f I\n"), ff(RPM_PID.GetKi()));
        zprintf(PSTR("EPR:3 324 %f D\n"), ff(RPM_PID.GetKd()));
#endif
        zprintf(PSTR("EPR:3 332 %f EXTADV\n"), ff(extadv));
        //zprintf(PSTR("EPR:3 336 %d UNMS\n"), fi(unms));
#ifdef WIFISERVER
        zprintf(PSTR("EPR:3 380 %d GCODE\n"), fi(wifi_gcode));
#endif
*/
        break;
#ifdef WIFISERVER
      // show wifi
      case 504:

        zprintf(PSTR("Wifi AP 400:%s PWD 450:%s mDNS 470:%s\n"), wifi_ap, wifi_pwd, wifi_dns);
        zprintf(PSTR("%d.%d.%d.%d\n"), fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]));
        //zprintf(PSTR("NGOPO TO !"));
        break;
      case 505:

        ESP.restart();
        //zprintf(PSTR("Wifi AP 400:%s PWD 450:%s mDNS 470:%s\n"), wifi_ap, wifi_pwd, wifi_dns);
        break;
#endif
#ifdef USE_EEPROM
#endif
      case 220:
        //? --- M220: Set speed factor override percentage ---
        if ( ! next_target.seen_S)
          break;
        // Scale 100% = 100
        f_multiplier = next_target.S * 0.01;
        MLOOP

        break;
      case 290: // m290 baby step in X Y Z E in milimeter
        if (next_target.seen_X) babystep[0] = next_target.target.axis[nX] * 4000;
        if (next_target.seen_Y) babystep[1] = next_target.target.axis[nY] * 4000;
        if (next_target.seen_Z) babystep[2] = next_target.target.axis[nZ] * 4000;
        if (next_target.seen_E) babystep[3] = next_target.target.axis[nE] * 4000;
        break;

      case 221:
        //? --- M220: Set speed factor override percentage ---
        if ( ! next_target.seen_S)
          break;
        // Scale 100% = 256
        e_multiplier = next_target.S * 0.01;
        MLOOP

        break;
      case 600: // change filament M600 Sxxx          S = length mm to unload filament, it will add 10mm when load, click endstop to resume
        changefilament(next_target.S);
        break;
        //      default:
        //zprintf(PSTR("E:M%d\nok\n"), next_target.M);
    } // switch (next_target.M)
  } // else if (next_target.seen_M)
} // process_gcode_command()

void init_gcode()
{
  next_target.target.F = 50;
  next_target.option_all_relative = 0;


}
