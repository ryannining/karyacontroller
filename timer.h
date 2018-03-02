#include "motion.h"

#include <stdint.h>
#define TMSCALE 1024L
extern int somedelay(int32_t n);
//#define somedelay(n) delayMicroseconds(n);
extern int feedthedog();
#ifndef ISPC
#define timescale 1000000L

#else
#define timescale 1000000L
extern uint32_t micros();

#endif
#define SUBMOTION 1
#define timescaleLARGE timescale*TMSCALE



extern uint32_t	next_step_time;
extern void timer_init();
extern void timer_stop();
extern void timer_reset();
extern uint8_t timer_set(int32_t delay);


#ifndef MASK
  /// MASKING- returns \f$2^PIN\f$
  #define MASK(PIN) (1 << PIN)
#endif


#ifdef USETIMER1
#ifdef __AVR__
#define MEMORY_BARRIER() __asm volatile( "" ::: "memory" );
#define CLI
//MEMORY_BARRIER() 
//cli();
#define SEI 
//sei();

#endif
#endif


#ifndef CLI
#define CLI
#define SEI
#define MEMORY_BARRIER()
#endif
