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
int busy1 = 0;
uint16_t ndelay;

#ifdef __AVR__
#define USETIMEROK


ISR(TIMER1_COMPA_vect)
//void tm()
{
    TIMSK1 &= ~(1 << OCIE1A);

    // stepper tick
    //cli();
    ndelay=20;
    coreloopm();
    TCNT1 =0;
    ndelay=fmax(20,ndelay);
    OCR1A=ndelay;
    //sei();
    TIMSK1 |= (1 << OCIE1A);
    //if (ndelay>40)zprintf(PSTR("%d\n"),fi(ndelay));
}

void timer_init()
{
    TCCR1A = 0;  // Steup timer 1 interrupt to no prescale CTC mode
    TIMSK1 = 0;
    TCNT1 =0;
    TCCR1B = (1 << CS11); // no prescaler == 0.0625 usec tick | 001 = clk/1
    OCR1A  =65500; //start off with a slow frequency.
    TIMSK1 |= (1<<OCIE1A); // Enable interrupt
}

#endif // avr timer
#ifdef __ARM__
#define USETIMEROK
//HardwareTimer timer1(2);

void tm()
{
    Timer2.pause();
    ndelay=20;
    coreloopm();
    ndelay=fmax(20,ndelay);
    Timer2.setPeriod(ndelay);
    Timer2.resume();
}

void timer_init()
{
    Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
    Timer2.setPeriod(1000000); // in microseconds
    Timer2.attachCompare1Interrupt(tm);
}

#endif // arm

void timer_set(uint16_t delay)
{
#ifdef output_enable
    zprintf(PSTR("TmSet %d\n"),fi(delay));
#endif
    ndelay = delay;
}

#endif


//#elif defined(__ARM__)//avr
#ifndef USETIMEROK
void timer_init() {};
void timer_set(int32_t delay) {};
#endif
