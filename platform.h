
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
#else
//#warning This is PC
#define ISPC
#define PROGMEM
#endif
