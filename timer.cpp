#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include <stdint.h>
#include "common.h"

int dogfeed=0;

#if defined(__AVR__) || defined(ESP8266)
    #include<arduino.h>
    #define dogfeedevery 100 // loop
// ESP8266 specific code here
#else
    #define dogfeedevery 100000 // loop
    #include<sys/time.h>
    uint32_t micros()
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        return 1000000 * tv.tv_sec +tv.tv_usec;
    }

#endif

void feedthedog(){
    if (dogfeed++> dogfeedevery) {
            dogfeed=0;
            #if defined(__AVR__)
            // AVR specific code here
            #elif defined(ARDUINO_ARCH_ESP8266)
            // ESP8266 specific code here
             ESP.wdtFeed();
            #else
            #endif
            //xprintf(PSTR("Feed the dog\n"));
        }
    }


