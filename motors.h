#include "motion.h"

#ifndef MOTOR_H
#define MOTOR_H
#include "config_pins.h"

// implement shift register pins here
byte pinData=0;
#define SR_data 1;
#define SR_clock 2;
#define SR_latch 3;

#define dWriteOn(pin) pinData|=1<<pin;
#define dWriteOff(pin) pinData&=!(byte)(1<<pin);

#define dWrite(pin,h) if(h)dWriteOn(pin);else dWriteOff(pin);
#define dShiftout {\
    digitalWrite(SR_latch, LOW);\
    shiftOut(SR_data, SR_clock, MSBFIRST, 1 << i);\
    digitalWrite(SR_latch, HIGH);\
}
 


#define DUMMYMOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){}\
  inline void motor_##AX##_ON(){}\
  inline void motor_##AX##_OFF() {}\
  inline void motor_##AX##_DIR(int d){}\
  inline void motor_##AX##_STEP(){}\
  inline void motor_##AX##_UNSTEP(){}

#ifndef ISPC

//    zprintf(PSTR("Backlash AX%d %d\n"),fi(AX),fi(PSTEP));\

#define MOTORBACKLASH(AX,d,PSTEP) \
  if (PSTEP && lsx[AX] && (lsx[AX]!=d)){\
    for(int i=0;i<PSTEP;i++){\
      motor_##AX##_STEP();\
      motor_##AX##_UNSTEP();\
      delayMicroseconds(500);\
    }\
  }\
  lsx[AX]=d;\

#define STEPDELAY delayMicroseconds(5);
#define STEPDIRDELAY delayMicroseconds(10);

#define MOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){pinMode(PENABLE, OUTPUT);pinMode(PDIR, OUTPUT);pinMode(PSTEP, OUTPUT);digitalWrite(PENABLE,1);}\
  inline void motor_##AX##_ON(){ digitalWrite(PENABLE,0);}\
  inline void motor_##AX##_OFF() { digitalWrite(PENABLE,1);}\
  inline void motor_##AX##_STEP(){  digitalWrite(PSTEP,1);STEPDELAY}\
  inline void motor_##AX##_UNSTEP(){  digitalWrite(PSTEP,0);}\
  inline void motor_##AX##_DIR(int d){ if(!d)return;digitalWrite(PENABLE,0);digitalWrite(PDIR,(d*MOTOR_##AX##_DIR)>0?1:0);STEPDIRDELAY;MOTORBACKLASH(AX,d,xback[AX]);}\

#else

    // PC just use dummy
#define MOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){zprintf(PSTR("Motor ##AX## Init\n"));}\
  inline void motor_##AX##_ON(){ zprintf(PSTR("Motor ##AX## Enable\n"));}\
  inline void motor_##AX##_OFF() { zprintf(PSTR("Motor ##AX## Disable\n"));}\
  inline void motor_##AX##_STEP(){  zprintf(PSTR("Motor ##AX## step\n"));}\
  inline void motor_##AX##_UNSTEP(){  zprintf(PSTR("Motor ##AX## unstep"\n));}\
  inline void motor_##AX##_DIR(int d){ zprintf(PSTR("Motor ##AX## Dir %d Backlash %d\n"),d,MOTOR_##AX##_BACKLASH);}\


#endif


#ifdef xenable
MOTOR(0, xenable, xdirection, xstep)
#else
DUMMYMOTOR(0, 0, 0, 0)
#endif

#ifdef yenable
MOTOR(1, yenable, ydirection, ystep)
#else
DUMMYMOTOR(1, 0, 0, 0)
#endif

#ifdef zenable
MOTOR(2, zenable, zdirection, zstep)
#else
DUMMYMOTOR(2, 0, 0, 0)
#endif

#ifdef e0enable
MOTOR(3, e0enable, e0direction, e0step)
#else
DUMMYMOTOR(3, 0, 0, 0)
#endif

#endif
