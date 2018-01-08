#include <stdint.h>

extern int feedthedog();
#if defined(__AVR__)
#define timescale 1000000L

// AVR specific code here
#elif defined(ESP8266)
#define timescale 1000000L
#else
#define timescale 1000000L
extern uint32_t micros();

#endif

#define timescale256 timescale*256L
