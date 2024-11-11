#pragma once
#ifndef motors_H
#define motors_H

#include<Arduino.h>
extern int mdir_pin[4];
extern int mstep_pin[4];

extern float stepmmy;
extern int dirpiny;
extern int steppiny;

//static int8_t lsx[4] = {0, 0, 0, 0};

#define dwrite(pin,v) digitalWrite(pin,v)
#define dread(pin) digitalRead(pin)
#define pinmode(pin,m) pinMode(pin,m)
#define pinMotorInit

#define STEPDELAY
#define STEPDIRDELAY

#define MOTOR(AX)\
  inline void motor_##AX##_INIT(){if (mdir_pin[AX]>-1){pinMode(mdir_pin[AX], OUTPUT);pinMode(mstep_pin[AX], OUTPUT);}}\
  inline void motor_##AX##_STEP(){if (mdir_pin[AX]>-1)dwrite(mstep_pin[AX],1);}\
  inline void motor_##AX##_UNSTEP(){if (mdir_pin[AX]>-1)dwrite(mstep_pin[AX],0);}\
  inline void motor_##AX##_DIR(int d){ if(d)dwrite(mdir_pin[AX],d>0);}\
/*
#define MOTOR(AX)\
  inline void motor_##AX##_INIT(){if (mdir_pin[AX]>-1){pinMode(mdir_pin[AX], OUTPUT);pinMode(mstep_pin[AX], OUTPUT);}}\
  inline void motor_##AX##_STEP(){dwrite(mstep_pin[AX],1);}\
  inline void motor_##AX##_UNSTEP(){dwrite(mstep_pin[AX],0);}\
  inline void motor_##AX##_DIR(int d){ if(d)dwrite(mdir_pin[AX],d>0);}\
*/
MOTOR(0)
MOTOR(1)
MOTOR(2)





#endif
