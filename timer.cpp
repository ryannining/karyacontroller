#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include <stdint.h>
#include "common.h"
#include "motion.h"


int somedelay(int32_t n) {
  float f = 0;
  int m=1;
  
  //while (m--) {
  int nn=n;
  while (nn--) {
    f +=n;
    asm("");
  }//}
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

int feedthedog() {
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
ISR(TIMER1_COMPA_vect) {
  if (busy1) {
    zprintf(PSTR("Busy %d\n"),fi(dl));
    return;
  }
  busy1 = 1;
  TIMSK1 &= ~(1 << OCIE1A);

  // stepper tick
  motionloop();
  busy1 = 0;
}

void timer_init() {
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TIMSK1 = 0;
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TCNT1  = 0;//initialize counter value to 0

}

void timer_stop() {
  // disable all interrupts
  TIMSK1 = 0;
}
void timer_reset() {
}
uint8_t timer_set(int32_t delay) {
  noInterrupts();  
  OCR1A = delay; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}
#endif

#endif


//#elif defined(__ARM__)//avr
#ifndef USETIMEROK
void timer_init() {};
void timer_stop() {};
void timer_reset() {};
uint8_t timer_set(int32_t delay) {};
#endif
