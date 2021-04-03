#include "motion.h"
#include "config_pins.h"

#include <stdint.h>
#define TMSCALE 1024L
extern int somedelay(int32_t n);
//#define somedelay(n) delayMicroseconds(n);
extern int feedthedog();

#define TEMPTICK 100000 //500ms
#define timescale 1000000L
#ifdef ISPC
extern uint32_t micros();
#else

#if defined(ESP8266) && defined(WIFISERVER)
#define usetmr1
#define TEMPTICK 100000 //100ms
#endif

extern uint32_t get_RPM();


#endif // ISPC

#define SUBMOTION 1
#define timescaleLARGE timescale*TMSCALE

extern void set_pwm(int v);
extern void pause_pwm(bool v);
extern volatile uint32_t ndelay;

extern uint32_t	next_step_time;
extern void timer_init();
extern  void timer_set(int32_t delay);
extern void servo_loop();
extern void servo_set(int angle);

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

#ifdef xESP32
extern void THEISR timerPause();
extern void THEISR timerResume();

#else
#define timerPause()
#define timerResume()
#endif
