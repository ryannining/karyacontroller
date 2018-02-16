#include "motion.h"

#include <stdint.h>
#define TMSCALE 1024L
extern int feedthedog();
#ifndef ISPC
#define timescale 1000000L

#else
#define timescale 1000000L
extern uint32_t micros();

#endif
#define SUBMOTION 1
#define timescaleLARGE timescale*TMSCALE
