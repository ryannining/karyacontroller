
#ifndef MOTION_H
#define MOTION_H

#ifdef ARDUINO
#include<arduino.h>
#endif

#if defined(STM32_MCU_SERIES) || defined(STM32_SERIES_F1)
#define __ARM__
#endif

#if defined(__AVR__) || defined(ESP8266)  || defined (__ARM__)
#else
//#warning This is PC
#define ISPC
#define PROGMEM
#endif


#include<math.h>
#include<stdint.h>
#include "config_pins.h"
#define NUMAXIS 4
#define UPDATE_F_EVERY 1600 //us


typedef struct {
  int8_t  sx[NUMAXIS];
  int8_t  status  ; // status in bit 01 , planstatus in bit 2 , g0 in bit 4, 4 bit left better use it for fast axis

  int fx[NUMAXIS];
  float fs, fn, fe;
  int32_t dx[NUMAXIS];
  int32_t rampup, rampdown;
  int32_t ac1, ac2;
#ifdef ISPC
  // for graphics
  int col;
  float xs[4];
#endif
} tmove;

extern uint8_t e_multiplier, f_multiplier;
#ifdef ISPC
extern float tick, tickscale, fscale, graphscale;
#endif
extern int32_t mcx[NUMAXIS];
extern tmove *m;
//extern int32_t px[4];
extern uint8_t xback[4];
extern uint8_t homingspeed;
extern uint8_t homeoffset[4];
extern uint8_t jerk[4];
extern int accel[4];
extern int mvaccel[4];
extern uint8_t maxf[4];
extern float stepmmx[4];
extern tmove move[NUMBUFFER];
extern float cx1, cy1, cz1, ce01;
extern uint8_t head, tail;
extern int8_t checkendstop;
extern int8_t endstopstatus[3];
extern uint16_t ax_max[3];
#define nextbuff(x) ((x) < NUMBUFFER-1 ? (x) + 1 : 0)
#define prevbuff(x) ((x) > 0 ? (x) - 1 : NUMBUFFER-1)

#define degtorad(x) x*22/(7*180);

/*
  static int32_t ramplen(float v0, float v1, float a , float stepmm);
  static float speedat(float v0, float a, float s, float stp);
  static float accelat(float v0, float v1, float s);
*/

extern void power_off();

extern int32_t motionrunning;
extern int motionloop();

extern int coreloop();
extern void waitbufferempty();
extern void needbuffer();
extern int32_t startmove();
extern void initmotion();
extern void addmove(float cf, float cx2, float cy2 , float cz2, float ce02 , int g0 = 1 , int rel = 0);
extern void homing();
extern int32_t bufflen();
extern void docheckendstop();
extern void reset_motion();



#endif

