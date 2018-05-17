
#ifndef MOTION_H
#define MOTION_H

#ifdef ARDUINO
#include<arduino.h>
#endif

#if defined(__STM32F1__) || defined(STM32_SERIES_F1)
#define __ARM__
#endif
#ifdef ESP32
#define ESP8266  
#endif

#if defined(__AVR__) || defined(ESP8266)|| defined(ESP32)  || defined (__ARM__)
#else
//#warning This is PC
#define ISPC
#define PROGMEM
#endif


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
  uint8_t cmd  ; // 1 bit CMD 0:setdir 1:step 4bit parameter for dir/step, 11 bits for delay , delay*10 0-20480, si min speed is 0.5mm/s for 100step/mm motor
  uint16_t dly  ; // 1 bit CMD 0:setdir 1:step 4bit parameter for dir/step, 11 bits for delay , delay*10 0-20480, si min speed is 0.5mm/s for 100step/mm motor
} tcmd;


typedef struct {
  int8_t  status  ; // status in bit 01 , planstatus in bit 2 , g0 in bit 4, 4 bit left better use it for fast axis
  float dis; // max start speed, maxcorner
#ifdef __AVR__
  int16_t ac; // needed for backplanner
  uint16_t fs, fn,maxs; // all are in square ! needed to calc real accell
#else
  float ac; // needed for backplanner
  float fs, fn,maxs; // all are in square ! needed to calc real accell
#endif
//#ifdef NONLINEAR
  //float otx[3]; // keep the original coordinate before transform
  float dtx[NUMAXIS]; // keep the original coordinate before transform
//#endif
  int32_t dx[NUMAXIS]; //original delta before transform
#ifdef ISPC
  // for graphics
  int col;
#endif
} tmove;

extern float e_multiplier, f_multiplier;
#ifdef ISPC
extern float tick, tickscale, fscale, graphscale;
#endif
extern int32_t mcx[NUMAXIS];
extern tmove *m;
//extern int32_t px[4];
extern int xback[4];
extern uint8_t homingspeed;
extern uint8_t homeoffset[4];
extern int xyjerk, accel;
extern int mvaccel;
extern int  maxf[4];
extern int32_t dlp, dl;
extern float stepmmx[4], xyscale;
extern tmove move[NUMBUFFER];
extern float cx1, cy1, cz1, ce01;
extern uint8_t head, tail;
extern int8_t checkendstop;
extern int16_t endstopstatus;
extern float ax_home[NUMAXIS];
//extern int8_t lsx[4];
extern int8_t  sx[NUMAXIS];
extern uint32_t cmctr;
extern int8_t RUNNING;
extern int8_t PAUSE;

#define nextbuff(x) ((x) < NUMBUFFER-1 ? (x) + 1 : 0)
#define prevbuff(x) ((x) > 0 ? (x) - 1 : NUMBUFFER-1)


#define degtorad(x) x*22/(7*180);


extern void power_off();

extern int32_t motionrunning;
extern int32_t mctr;
extern int motionloop();
extern int laserOn;
extern void init_pos();
extern int coreloop();
extern void coreloopm();
extern void otherloop(int r);
extern void waitbufferempty();
extern void needbuffer();
extern int32_t startmove();
extern void initmotion();
extern void addmove(float cf, float cx2, float cy2, float cz2, float ce02, int g0 = 1, int rel = 0);
extern void draw_arc(float cf, float cx2, float cy2, float cz2, float ce02, float fI, float fJ, uint8_t isclockwise);

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
extern int32_t bufflen();
void docheckendstop();
extern void reset_motion();
extern void preparecalc();
extern tmove* m;
#define fmax(a,b) a<b?b:a
#define fmin(a,b) a>b?b:a

#define domotionloop motionloop();

#endif
