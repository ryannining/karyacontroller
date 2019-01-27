
#ifdef ARDUINO
#include<Arduino.h>
#endif
// if platformio cannot detect then force it
//#define STM32
//#define ESP32
//#define ESP8266


#if defined(STM32) || defined(__STM32F1__) || defined(STM32_SERIES_F1)
#define __ARM__
#endif

#ifdef ESP32
#define ESP8266
#endif

#if defined(__AVR__) || defined(ESP8266)|| defined(ESP32)  || defined (__ARM__)
extern int CNCMODE;
#define LASER(x) if (!CNCMODE)digitalWrite(laser_pin,x); 
#define SPINDLE(x) if (CNCMODE)digitalWrite(laser_pin,x); 

#else
//#warning This is PC
#define LASER(x) {}
#define ISPC
#define PROGMEM
#endif
