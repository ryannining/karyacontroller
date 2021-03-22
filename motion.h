


#ifndef MOTION_H
#define MOTION_H


#define UPDATE_V_EVERY 4 // must be 1<<n  16 =1<<4
#define DELAYBETWEENSTEP 3
#define nX 0
#define MX 0
#define nE 3

// Corner deviation Setting
//#define FASTBUFFERFILL 10 // if need faster buffer filling.
//#define FASTBUFFERFILL2 10 // if need faster buffer filling.
// Centripetal
#define MINCORNERSPEED 4 // minimum cornering speed
#define MINSTEP 0
//#define TSTEP 0.0005 // time stepping to get the velocity
#define TSTEP 0.001 // time stepping to get the velocity

#ifdef DRIVE_XYYZ
  #define MZ 1
  #define MY 2
  #define nY 2
  #define nZ 1
#else
  #define MZ 2
  #define MY 1
  #define nY 1
  #define nZ 2
#endif

#define D_CARTESIAN 0
#define D_COREXY 1
#define D_COREXZ 2
#define D_XYYZ 3

#include "platform.h"
#include<math.h>
#include<stdint.h>
#include "config_pins.h"
#define NUMAXIS 4

#ifdef __AVR__
#define LOWESTDELAY 540 // IF LESS than this microsec then do the subpixel
#else
#define LOWESTDELAY 120 // IF LESS than this microsec then do the subpixel
#endif


typedef struct {
  int8_t  status  ; // status in bit 01 , planstatus in bit 2 , g0 in bit 4, 4 bit left better use it for fast axis
  int laserval;
  float dis; // max start speed, maxcorner
#ifdef __AVR__
  int16_t ac; // needed for backplanner
  uint16_t fs, fn, delta, maxs; // all are in square ! needed to calc real accell
#else
  int32_t ac, delta, maxs; // needed for backplanner
  int32_t fs, fn; // all are in square ! needed to calc real accell
#endif
  int32_t dx[NUMAXIS]; //original delta before transform
  float dtx[NUMAXIS]; // keep the original coordinate before transform

#ifdef ISPC
  // for graphics
  int col;
#endif
} tmove;


#ifdef MESHLEVEL

extern int XCount, YCount;
extern int ZValues[40][40];

extern float pointProbing();
extern int MESHLEVELING;
#endif

extern float e_multiplier, f_multiplier;
#ifdef ISPC
extern float tick, tickscale, fscale, graphscale;
#endif
extern int vSUBPIXELMAX;
extern int32_t mcx[NUMAXIS];
extern tmove *m;
extern int babystep[4];
extern int32_t e_ctr;
extern int mm_ctr;
//extern int32_t px[4];
extern int xback[4];
extern uint8_t homingspeed;
extern uint8_t homeoffset[4];
extern int32_t xyjerk, zjerk, xycorner, zcorner;
extern int zaccel, accel;
extern int  maxf[4];
extern int  maxa[4];
extern int32_t dlp;
extern float stepmmx[4], Lscale;
extern float retract_in, retract_out;
extern float info_x,info_y,info_z,info_e,perstepx,perstepy,perstepz;
extern float retract_in_f, retract_out_f;
extern tmove moves[NUMBUFFER];
extern float cx1, cy1, cz1, ocz1, ce01;
extern uint8_t head, tail;
extern int8_t checkendstop;
extern int16_t endstopstatus;
extern float ax_home[NUMAXIS];
//extern int8_t lsx[4];
extern int8_t  sx[NUMAXIS];
extern uint32_t cmctr;
extern int8_t RUNNING;
extern int8_t PAUSE;
extern int constantlaserVal;
extern float extadv;
extern String hstatus;
#define nextbuff(x) ((x) < NUMBUFFER-1 ? (x) + 1 : 0)
#define prevbuff(x) ((x) > 0 ? (x) - 1 : NUMBUFFER-1)
extern float Interpolizer(int zX, int zY);


#define degtorad(x) x*22/(7*180);


extern void power_off();
extern uint32_t ectstep;
extern int32_t motionrunning;
extern int32_t mctr;
extern int motionloop();
extern int laserOn,home_cnt;
extern void init_pos();
extern int coreloop();
extern void coreloopm();
extern void otherloop(int r);
extern void waitbufferempty();
extern void needbuffer();
extern int32_t startmove();
extern void initmotion();
#ifdef ARC_SUPPORT

extern void draw_arc(float cf, float cx2, float cy2, float cz2, float ce02, float fI, float fJ, uint8_t isclockwise);
#endif
extern void addmove(float cf, float cx2, float cy2, float cz2, float ce02, int g0, int rel);

extern float axisofs[4];

#ifdef NONLINEAR
extern float delta_diagonal_rod;
extern float DELTA_DIAGONAL_ROD_2;
extern float delta_radius;


// Compile-time trigonometric functions. See Taylor series.
// Converts degrees to radians.
#define DegToRad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
//Make an angle (in radian) coterminal (0 >= x > 2pi).
//#define COTERMINAL(x) (fmod((x), M_PI*2)) //This causes error since fmod() implementation is different from i386 compiler.
//TODO: Solve coterminality
#define COTERMINAL(x) (x)
// Get quadrant of an angle in radian.
#define QUADRANT(x) ((uint8_t)((COTERMINAL((x))) / M_PI_2) + 1)
//Calculate sine of an angle in radians.
//Formula will be adjusted accordingly to the angle's quadrant.
//Don't worry about pow() function here as it will be optimized by the compiler.
#define SIN0(x) (x)
#define SIN1(x) (SIN0(x) - (pow ((x), 3) / 6))
#define SIN2(x) (SIN1(x) + (pow ((x), 5) /  120))
#define SIN3(x) (SIN2(x) - (pow ((x), 7) /  5040))
#define SIN4(x) (SIN3(x) + (pow ((x), 9) /  362880))
//*
#define SIN(x) (QUADRANT((x)) == 1 ? (SIN4(COTERMINAL((x)))) : \
                (QUADRANT((x)) == 2 ? (SIN4(M_PI - COTERMINAL((x)))) : \
                 (QUADRANT((x)) == 3 ? -(SIN4(COTERMINAL((x)) - M_PI)) : \
                  -(SIN4(M_PI*2 - COTERMINAL((x)))))))
//               */
//#define SIN(x) (SIN4(x))
//Calculate cosine of an angle in radians.
//Formula will be adjusted accordingly to the angle's quadrant.
//Don't worry about pow() function here as it will be optimized by the compiler.
#define COS0(x) 1
#define COS1(x) (COS0(x) - (pow ((x), 2) /  2))
#define COS2(x) (COS1(x) + (pow ((x), 4) /  24))
#define COS3(x) (COS2(x) - (pow ((x), 6) /  720))
#define COS4(x) (COS3(x) + (pow ((x), 8) /  40320))
//*
#define COS(x) (QUADRANT((x)) == 1 ? (COS4(COTERMINAL((x)))) : \
                (QUADRANT((x)) == 2 ? -(COS4(M_PI - COTERMINAL((x)))) : \
                 (QUADRANT((x)) == 3 ? -(COS4(COTERMINAL((x)) - M_PI)) : \
                  (COS4(M_PI*2 - COTERMINAL((x)))))))
//               */
//#define COS(x) COS4((x))
//Must scale by 16 because 2^32 = 4,294,967,296 - giving a maximum squareroot of 65536
//If scaled by 8, maximum movement is 65,536 * 8 = 524,288um, 65,536 * 16 = 1,048,576um

#endif


#define amove addmove


extern void homing();

void docheckendstop(int m);
extern void reset_motion();
extern void preparecalc();
extern tmove* m;
#define fmax(a,b) a<b?b:a
#define fmin(a,b) a>b?b:a
#define fmax3(a,b,c) fmax(a,fmax(b,c))
#define fmin3(a,b,c) fmin(a,fmin(b,c))
#define bufflen (head >= tail ? head - tail : (NUMBUFFER + head) - tail)

#define domotionloop motionloop();

#endif
