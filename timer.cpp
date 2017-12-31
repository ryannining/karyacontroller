#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include <stdint.h>

int dogfeed=0;

#if defined(__AVR__)
// AVR specific code here
    #define dogfeedevery 100 // step
    #include<arduino.h>
#elif defined(ESP8266)
    #include<arduino.h>
    #define dogfeedevery 100 // step
// ESP8266 specific code here
#else
    #define dogfeedevery 10000 // step
    #include<time.h>
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
             Serial.print("Feed the dog\n");
            #elif defined(ARDUINO_ARCH_ESP8266)
            // ESP8266 specific code here
             ESP.wdtFeed();
             Serial.print("Feed the dog\n");
            #else
            printf("Feed the dog\n");
            #endif
        }
    }


