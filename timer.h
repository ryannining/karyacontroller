#pragma once
#ifndef timer_H
#define timer_H


#include <stdint.h>
#define TMSCALE 1024L
#define MINDELAY 50
extern void somedelay(int32_t n);
//#define somedelay(n) delayMicroseconds(n);
extern void feedthedog();


#define timescale 1000000L
#ifdef ESP32
    extern portMUX_TYPE timerMux;
    #define ENTERCRITICAL portENTER_CRITICAL_ISR(&timerMux);
    #define EXITCRITICAL portEXIT_CRITICAL_ISR(&timerMux);
#else
    #define ENTERCRITICAL 
    #define EXITCRITICAL 
    
#endif

extern uint32_t get_RPM();


#define SUBMOTION 1
#define timescaleLARGE timescale*TMSCALE

extern void set_tool(int v);
extern void pause_pwm(bool v);
extern volatile uint32_t ndelay;


extern uint32_t	next_step_time;
extern void timer_init();
extern  void THEISR timer_set(int32_t delay);
//extern void THEISR timer_set2(int32_t delay1,int32_t delay2);
extern void servo_loop();
extern void servo_set(int angle);



#endif
