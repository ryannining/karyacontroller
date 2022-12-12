//#pragma once
#ifndef platform_H
#define platform_H

#ifdef ARDUINO
#include<Arduino.h>
#endif
// if platformio cannot detect then force it
//#define STM32
//#define ESP32
//#define ESP8266


#ifdef ESP8266
#define D0    16
#define  D1   5
#define  D2   4
#define  D3   0
#define  D4   2
#define  D5   14
#define  D6   12
#define  D7   13
#define  D8   15
#define  D9   3
#define  D10  1
#define  TX   1
#define  RX  3
#endif

#ifdef ESP32
#warning Detected ESP32
//#define ESP8266
#endif

#if  defined(ESP32)  || defined (ESP8266)

extern int HEATING;
//#include "motors.h"
extern int atool_pin,pwm_pin;
extern bool TOOLON;
extern bool TOOLONS[3];

#define TOOL1(x) {xdigitalWrite(atool_pin,x);}
#define TOOLPWM(x) {xdigitalWrite(pwm_pin,x);}

#else
#warning Unsupported CPU

#endif

#endif
