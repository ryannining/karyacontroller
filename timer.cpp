#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include <stdint.h>
#include "common.h"
#include "motion.h"


int somedelay(int32_t n)
{
    float f = 0;
    int m=1;

    while (m--) {
        int nn=n;
        while (nn--) {
            f +=n;
            asm("");
        }
    }
    return f + n;
}

//#define somedelay(n) delayMicroseconds(n);
int dogfeed = 0;

#ifndef ISPC
#include<arduino.h>
#define dogfeedevery 200 // loop
// ESP8266 specific code here
#else
#define dogfeedevery 100000 // loop
#include<sys/time.h>
uint32_t micros()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return 1000000 * tv.tv_sec + tv.tv_usec;
}

#endif

int feedthedog()
{
    if (dogfeed++ > dogfeedevery) {
        dogfeed = 0;
#if defined(__AVR__)
        // AVR specific code here
#elif defined(ESP8266)
        // ESP8266 specific code here
        ESP.wdtFeed();
#else
#endif
        //xprintf(PSTR("Feed the dog\n"));
        return 1;
    }
    return 0;
}


// ======================== TIMER for motionloop =============================

uint32_t	next_step_time;
#ifdef USETIMER1
#ifdef __AVR__
#define USETIMEROK
int busy1 = 0;
uint16_t ndelay;
ISR(TIMER1_COMPA_vect)
{
    if (busy1) {
#ifdef output_enable
        zprintf(PSTR("Busy %d\n"),fi(delay));
#endif
        zprintf(PSTR("Busy %d\n"),fi(delay));
        return;
    }
    busy1 = 1;
    CLI
    //TIMSK1 &= ~(1 << OCIE1A);
    ndelay =240;
    // stepper tick
    coreloopm();
    TCNT1 =0;
    OCR1A=(ndelay);
    //TIMSK1 |= (1 << OCIE1A);
    busy1 = 0;
    //zprintf(PSTR("%d\n"),fi(ndelay));
    SEI
}

 void timer_set(uint16_t delay)
{
#ifdef output_enable
    zprintf(PSTR("TmSet %d\n"),fi(delay));
#endif    
    ndelay = delay;
}
void timer_init()
{
    TCCR1A = 0;  // Steup timer 1 interrupt to no prescale CTC mode
    TIMSK1 = 0;
    TCNT1 =0;
    TCCR1B = (1 << CS10); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A  =65500; //start off with a slow frequency.
    TIMSK1 |= (1<<OCIE1A); // Enable interrupt
}

#endif

#endif


//#elif defined(__ARM__)//avr
#ifndef USETIMEROK
    void timer_init() {};
     void timer_set(int32_t delay) {};
#endif
