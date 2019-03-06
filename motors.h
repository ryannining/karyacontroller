
#include "motion.h"

#ifndef MOTOR_H
#define MOTOR_H
#include "config_pins.h"
#ifndef ISPC
#include<Arduino.h>
#else
#include <stdlib.h>
#endif

static int8_t lsx[4] = {0, 0, 0, 0};



#ifndef ISPC



#if defined(USEDIO) && defined(__AVR__)
#include "DIO2.h"
#define dwrite(pin,v) digitalWrite2(pin,v)
#define dread(pin) digitalRead2(pin)
#define pinmode(pin,m) pinMode2(pin,m)
#else
#define dwrite(pin,v) digitalWrite(pin,v)
#define dread(pin) digitalRead(pin)
#define pinmode(pin,m) pinMode(pin,m)
#endif
#endif //ispc



#ifdef USE_SHIFTREG

static byte srdata = 0;

#define xdigitalWrite(pin,v) if(v)srdata|=1 << pin;else srdata &=~(1 << pin);

//#define pinCommit() digitalWrite(pinlatch,LOW);shiftOut(pindata,pinclock,MSBFIRST,srdata);digitalWrite(pinlatch,HIGH);
//Fast shiftout ESP8266 D0 as LATCH
#ifdef ESP8266
#define pinH(pin) GPOS = (1 << pin)
#define pinL(pin) GPOC = (1 << pin)
static void pinCommit() {
  // USE D0 as latch
  GP16O |= 1;
#define pushbit(i)if (srdata&i)pinH(pindata);else pinL(pindata);pinL(pinclock);pinH(pinclock);
  //GP16O |= 1;GP16O &= ~1;
  for (int i = 7; i >= 0; i--)  {
    pushbit(1 << i);
  }
  GP16O &= ~1;
}
#endif

#define xpinMode(p,v)
#define pinMotorInit     pinMode(pindata,OUTPUT);pinMode(pinlatch,OUTPUT);pinMode(pinclock,OUTPUT);

#else
#define xpinMode(p,v) pinmode(p,v)
#define xdigitalWrite(p,v) dwrite(p,v)
#define pinMotorInit
#define pinCommit()
#endif

#define DUMMYMOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){}\
  inline void motor_##AX##_INIT2(){}\
  inline void motor_##AX##_ON(){}\
  inline void motor_##AX##_OFF() {}\
  inline void motor_##AX##_DIR(int d){}\
  inline void motor_##AX##_STEP(){}\
  inline void motor_##AX##_UNSTEP(){}

#ifndef ISPC


static uint8_t bsteps = 0;

#define STEPDELAY
//somedelay(10);
#define STEPDIRDELAY
//somedelay(10);

#define MOTOR(AX,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){xpinMode(PDIR, OUTPUT);xpinMode(PSTEP, OUTPUT);}\
  inline void motor_##AX##_STEP(){  xdigitalWrite(PSTEP,1);}\
  inline void motor_##AX##_UNSTEP(){  xdigitalWrite(PSTEP,0);}\
  inline void motor_##AX##_DIR(int d){ if(!d)return;motor_##AX##_ON();xdigitalWrite(PDIR,d>0);STEPDIRDELAY;}\

#define MOTOREN(AX,PENABLE)\
  inline void motor_##AX##_ON() { xdigitalWrite(PENABLE,0);}\
  inline void motor_##AX##_OFF() { xdigitalWrite(PENABLE,1);}\
  inline void motor_##AX##_INIT2() {xpinMode(PENABLE, OUTPUT);xdigitalWrite(PENABLE,1);}\

#define MOTOREN0(AX)\
  inline void motor_##AX##_ON(){}\
  inline void motor_##AX##_OFF() {}\
  inline void motor_##AX##_INIT2() {}\

  /*
  #else
  #define STEPDELAY
  #define STEPDIRDELAY

// PC just use dummy
#define MOTOR(AX,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){zprintf(PSTR("Motor ##AX## Init\n"));}\
  inline void motor_##AX##_STEP(){  zprintf(PSTR("Motor ##AX## step\n"));}\
  inline void motor_##AX##_UNSTEP(){  zprintf(PSTR("Motor ##AX## unstep"\n));}\
  inline void motor_##AX##_DIR(int d){ zprintf(PSTR("Motor ##AX## Dir %d\n"),d);}\

#define MOTOREN(AX,PENABLE)\
  inline void motor_##AX##_ON(){ zprintf(PSTR("Motor ##AX## Enable\n"));}\
  inline void motor_##AX##_OFF() { zprintf(PSTR("Motor ##AX## Disable\n"));}\

  */
#endif


#ifdef xstep
#ifdef xenable
MOTOREN(0, xenable)
#else
MOTOREN0(0)
#endif
MOTOR(0,  xdirection, xstep)
#else
DUMMYMOTOR(0, 0, 0, 0)
#endif

#ifdef ystep
#ifdef yenable
MOTOREN(1, yenable)
#else
MOTOREN0(1)
#endif
MOTOR(1,  ydirection, ystep)
#else
DUMMYMOTOR(1, 0, 0, 0)
#endif



#ifdef zstep
#ifdef zenable
MOTOREN(2, zenable)
#else
MOTOREN0(2)
#endif
MOTOR(2, zdirection, zstep)
#else
DUMMYMOTOR(2, 0, 0, 0)
#endif

#ifdef e0step
#ifdef e0enable
MOTOREN(3, e0enable)
#else
MOTOREN0(3)
#endif
MOTOR(3, e0direction, e0step)
#else
DUMMYMOTOR(3, 0, 0, 0)
#endif

#ifndef ISPC
#endif



#endif //mototh

