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
#ifdef ESP8266
//#ifdef WIFISERVER
#define usetmr1
//#endif
#endif

#endif

#define SUBMOTION 1
#define timescaleLARGE timescale*TMSCALE



extern uint32_t	next_step_time;
extern void timer_init();
extern  void timer_set(uint32_t delay);
extern  void timer_set2(uint32_t delay);
extern void servo_loop();
extern void servo_init();
extern void servo_set(int us);

#ifndef MASK
  /// MASKING- returns \f$2^PIN\f$
  #define MASK(PIN) (1 << PIN)
#endif


#ifdef USETIMER1
#ifdef __AVR__
#define MEMORY_BARRIER() __asm volatile( "" ::: "memory" );
#define CLI cli();
#define SEI sei();

#define ATOMIC_START { \
                       uint8_t save_reg = SREG; \
                       cli(); \
                       MEMORY_BARRIER();

#define ATOMIC_END   MEMORY_BARRIER(); \
                     SREG = save_reg; \
                   }
#endif
#endif


#ifndef CLI
#define CLI
#define SEI
#define MEMORY_BARRIER()
#define ATOMIC_START
#define ATOMIC_END
#endif
