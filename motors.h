
#ifndef motors_H
#define motors_H

#include<Arduino.h>
extern int mdir_pin[4];
extern int mstep_pin[4];


//static int8_t lsx[4] = {0, 0, 0, 0};

#define dwrite(pin,v) digitalWrite(pin,v)
#define dread(pin) digitalRead(pin)
#define pinmode(pin,m) pinMode(pin,m)


#ifdef USE_SHIFTREG

static byte srdata = 0;

#define xdigitalWrite(pin,v) if(v)srdata|=1 << pin;else srdata &=~(1 << pin);

//#define pinCommit() digitalWrite(pinlatch,LOW);shiftOut(pindata,pinclock,MSBFIRST,srdata);digitalWrite(pinlatch,HIGH);
//Fast shiftout ESP8266 D0 as LATCH

#ifdef ESP8266
#define pinH(pin) GPOS = (1 << pin)
#define pinL(pin) GPOC = (1 << pin)
static ThEISR void pinCommit() {
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


#define STEPDELAY
#define STEPDIRDELAY

#define MOTOR(AX)\
  inline void motor_##AX##_INIT(){if (mdir_pin[AX]>-1){xpinMode(mdir_pin[AX], OUTPUT);xpinMode(mstep_pin[AX], OUTPUT);}}\
  inline void motor_##AX##_STEP(){if (mdir_pin[AX]>-1)xdigitalWrite(mstep_pin[AX],1);}\
  inline void motor_##AX##_UNSTEP(){if (mdir_pin[AX]>-1)xdigitalWrite(mstep_pin[AX],0);}\
  inline void motor_##AX##_DIR(int d){ if(d)xdigitalWrite(mdir_pin[AX],d>0);}\
/*
#define MOTOR(AX)\
  inline void motor_##AX##_INIT(){if (mdir_pin[AX]>-1){xpinMode(mdir_pin[AX], OUTPUT);xpinMode(mstep_pin[AX], OUTPUT);}}\
  inline void motor_##AX##_STEP(){xdigitalWrite(mstep_pin[AX],1);}\
  inline void motor_##AX##_UNSTEP(){xdigitalWrite(mstep_pin[AX],0);}\
  inline void motor_##AX##_DIR(int d){ if(d)xdigitalWrite(mdir_pin[AX],d>0);}\
*/
MOTOR(0)
MOTOR(1)
MOTOR(2)





#endif
