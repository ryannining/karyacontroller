#include "IRremote.h"

//==============================================================================
// Static Data
//==============================================================================

// Protocol temporary data
volatile uint8_t CHashIR::count = 0;
uint32_t CHashIR::hash = FNV_BASIS_32;
volatile uint16_t CHashIR::lastDuration = 0xFFFF;
