#include "common.h"
#include "motion.h"
#include "timer.h"


#ifndef ISPC
// functions for sending decimal
#include<arduino.h>
#include <stdarg.h>

#define write_uint8(v, w)  write_uint32(v, w)
#define write_int8(v, w)   write_int32(v, w)
#define write_uint16(v, w) write_uint32(v, w)
#define write_int16(v, w)  write_int32(v, w)

// send one character


/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
//const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/


void write_uint32(uint32_t v) {
  uint8_t e, t;

  for (e = DECFLOAT_EXP_MAX; e > 0; e--) {
    if (v >= POWERS(e))
      break;
  }

  do
  {
    for (t = 0; v >= POWERS(e); v -= POWERS(e), t++);
    Serial.write(t + '0');
  }
  while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void write_int32(int32_t v) {
  if (v < 0) {
    Serial.write('-');
    v = -v;
  }

  write_uint32(v);
}

/** write decimal digits from a long unsigned int
  \param v number to send
  \param fp number of decimal places to the right of the decimal point
*/
void write_uint32_vf(uint32_t v, uint8_t fp) {
  uint8_t e, t;

  for (e = DECFLOAT_EXP_MAX; e > 0; e--) {
    if (v >= POWERS(e))
      break;
  }

  if (e < fp)
    e = fp;

  do
  {
    for (t = 0; v >= POWERS(e); v -= POWERS(e), t++);
    Serial.write(t + '0');
    if (e == fp)
      Serial.write('.');
  }
  while (e--);
}

/** write decimal digits from a long signed int
  \param v number to send
  \param fp number of decimal places to the right of the decimal point
*/
void write_int32_vf(int32_t v, uint8_t fp) {
  if (v < 0) {
    Serial.write('-');
    v = -v;
  }

  write_uint32_vf(v, fp);
}


#if __SIZEOF_INT__ == 2
#define GET_ARG(T) (va_arg(args, T))
#elif __SIZEOF_INT__ >= 4
#define GET_ARG(T) (va_arg(args, T))
#else
#define GET_ARG(T) (va_arg(args, T))
#endif

void sendf_P(PGM_P format_P, ...) {
  CLI
  va_list args;
  va_start(args, format_P);

  uint16_t i = 0;
  uint8_t c = 1, j = 0;
  while ((c = pgm_read_byte(&format_P[i++]))) {
    if (j) {
      switch (c) {
        case 'd':
          write_int32(GET_ARG(int32_t));
          j = 0;
          break;
        case 'f':
        case 'q':
          write_int32_vf((int32_t)GET_ARG(uint32_t), 3);
          j = 0;
          break;
        default:
          Serial.write(c);
          j = 0;
          break;
      }
    }
    else {
      if (c == '%') {
        j = 4;
      }
      else {
        Serial.write(c);
      }
    }
  }
  va_end(args);
  SEI
}
#endif

