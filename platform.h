#pragma once

#ifdef ARDUINO
#include<Arduino.h>
#endif
// if platformio cannot detect then force it
//#define STM32
//#define ESP32
//#define ESP8266


#if defined(STM32) || defined(__STM32F1__) || defined(__STM32F0__) || defined(STM32_SERIES_F1)
#undef _VARIANT_ARDUINO_STM32_
#warning Detected ARM
#define __ARM__
#endif

#if defined(_VARIANT_ARDUINO_STM32_)
#define __ARM__
#endif

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

#if defined(__AVR__) || defined(ESP8266)|| defined(ESP32)  || defined (__ARM__)

extern int HEATING;
#include "motors.h"
#ifdef laser_pin
#define LASER(x) {xdigitalWrite(laser_pin,x);}
#else
#define LASER(x) {}
#endif


#ifdef spindle_pin
#define SPINDLE(v) {set_pwm(v); }
#else
#define SPINDLE(v) {}
#endif

#ifdef heater_pin
#define HEATER(x) {xdigitalWrite(heater_pin,x);}
#else
#define HEATER(x) {}
#endif
#else
#warning Detected as PC
#define LASER(x) {}
#define ISPC
#define PROGMEM
#endif
