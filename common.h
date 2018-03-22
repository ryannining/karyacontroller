#ifndef COMMON_H
#define COMMON_H
#include "motion.h"
//const float powers[] = {1.f, 0.1f, 0.01f,0.001f,0.0001f, 0.00001f, 0.000001f, 0.0000001f, 0.00000001f, 0.000000001f};;
//#define POWERS(e) (powers[e])
//#define POWERS(e) (float)pgm_read_float(&(powers[e]))
const int32_t PROGMEM powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};;
#define POWERS(e) (int32_t)pgm_read_dword(&(powers[e]))
#define DECFLOAT_EXP_MAX 7
#ifndef ISPC
//#define output_enable
// AVR specific code here
//#include <avr/pgmspace.h>
#include <arduino.h>



void sendf_P(PGM_P format_P, ...);
// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.


//#define xprintf(...) sendf_P(serial_writechar, __VA_ARGS__)
//#define sersendf_P(...) sendf_P(serial_writechar, __VA_ARGS__)
#ifdef output_enable
#define xprintf   sendf_P
#define sersendf_P sendf_P
#endif

#define zprintf   sendf_P

#define ff(f) int32_t(1000.f*f)
#define fg(f) int32_t(100.f*f)
#define fi(f) (int32_t)f


#else // ispc
#define output_enable
#include<stdio.h>
#include<stdint.h>
#define PROGMEM
#define PGM_P const char *
#define PSTR(s) ((const PROGMEM char *)(s))
#define pgm_read_byte(x) (*((uint8_t *)(x)))
#define pgm_read_word(x) (*((uint16_t *)(x)))
#define pgm_read_dword(x) (*((uint32_t *)(x)))
#define pgm_read_float(x) (*((float *)(x)))

static void serial_writechar(uint8_t data) {
  printf("%c", (char)data);
}

#define ff(f) (float(f))
#define fg(f) (int32_t(f))
#define fi(f) int32_t(f)
#define xprintf printf
#define zprintf printf
#define sersendf_P printf
#endif


#endif // common_h
