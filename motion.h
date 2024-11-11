#pragma once
#ifndef motion_H
#define motion_H
#include "config_pins.h"

#define UPDATE_V_EVERY 3 // must be 1<<n  16 =1<<4
#define DELAYBETWEENSTEP 3
#define nX 0
#define MX 0
#define nE 3

// Corner deviation Setting

// Centripetal
#define MINCORNERSPEED 0.25 // minimum cornering speed
#define MINSTEP 0

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

//#include "platform.h"
#include<math.h>
#include<stdint.h>
//#include "config_pins.h"
#define NUMAXIS 4


#define LOWESTDELAY 50 // IF LESS than this microsec then do the subpixel



typedef struct {
  int8_t  status  ; // status in bit 01 , planstatus in bit 2 , g0 in bit 4, 4 bit left better use it for fast axis
  uint8_t maxv; 
  int laserval;
  float dis; // max start speed, maxcorner

  int32_t ac, delta, maxs; // needed for backplanner
  int32_t fs, fn,fr; // all are in square ! needed to calc real accell

  int32_t dx[NUMAXIS]; //original delta before transform
  //  float dtx[NUMAXIS]; // keep the original coordinate before transform
  
} tmove;


#ifdef MESHLEVEL

extern int XCount, YCount;
extern int ZValues[40][40];

extern float pointProbing(float floatdis);
extern int MESHLEVELING;
#endif

extern float e_multiplier, f_multiplier;

extern int mdir_pin[4];
extern int mstep_pin[4];
extern int lasermode;

extern int32_t mcx[NUMAXIS];
extern tmove *m;
extern int babystep[4];
extern int32_t e_ctr;
extern int mm_ctr;
//extern int32_t px[4];
extern float xback[4];
extern uint8_t homingspeed;
extern uint8_t homeoffset[4];
extern int32_t xycorner;
extern int zaccel, accel,limit_pin;
extern int  maxf[4];
extern int  maxa[4];
extern int32_t dlp, info_x_s, info_y_s, info_z_s;
extern float stepmmx[4];
extern float skew_y;  // y skew/mm X
extern float Lscale;

extern float info_x, info_y, info_z, info_ve, info_e, perstepx, perstepy, perstepz;
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
extern int volatile cmhead, cmtail;
extern int cmdlaserval;

#define degtorad(x) x*22/(7*180);


extern void power_off();
extern uint32_t ectstep;
extern int32_t motionrunning;
extern int32_t mctr;
extern int motionloop();
extern int laserOn, home_cnt;
extern void init_pos();
extern int coreloop();
extern void coreloopm();
extern void otherloop(int r);
extern void waitbufferempty(bool fullspeed=true);
//extern void needbuffer();
extern int32_t startmove();
extern void initmotion();
#ifdef ARC_SUPPORT

extern void draw_arc(float cf, float cx2, float cy2, float cz2, float ce02, float fI, float fJ, uint8_t isclockwise);
#endif
extern void addmove(float cf, float cx2, float cy2, float cz2, float ce02, int g0, int rel);

extern float axisofs[4];



#define amove addmove


extern void homing();

void docheckendstop(int m);
extern void reset_motion();
extern void pre_motion_set();
extern tmove* m;
#define fmax(a,b) ((a<b)?b:a)
#define fmin(a,b) ((a>b)?b:a)

#define fmax3(a,b,c) fmax(a,fmax(b,c))
#define fmin3(a,b,c) fmin(a,fmin(b,c))
#define bufflen (head >= tail ? head - tail : (NUMBUFFER + head) - tail)

#define domotionloop motionloop();

#endif
