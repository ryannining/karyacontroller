#include "common.h"
#include "timer.h"


// functions for sending decimal
#include<Arduino.h>
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


void write_uint32(void (*writechar)(uint8_t), uint32_t v) {
  uint8_t e, t;

  for (e = DECFLOAT_EXP_MAX; e > 0; e--) {
    if (v >= POWERS(e))
      break;
  }

  do
  {
    for (t = 0; v >= POWERS(e); v -= POWERS(e), t++);
    writechar(t + '0');
  }
  while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void write_int32(void (*writechar)(uint8_t), int32_t v) {
  if (v < 0) {
    writechar('-');
    v = -v;
  }

  write_uint32(writechar, v);
}
/** write decimal digits from a long signed int
  \param v number to send
*/
void write_char(void (*writechar)(uint8_t), char* v) {
  int l = strlen(v);
  if (l > 25)l = 25;
  for (int i = 0; i < l; i++) {
    writechar(v[i]);
  }
}

/** write decimal digits from a long unsigned int
  \param v number to send
  \param fp number of decimal places to the right of the decimal point
*/
void write_uint32_vf(void (*writechar)(uint8_t), uint32_t v, uint8_t fp) {
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
    writechar(t + '0');
    if (e == fp)
      writechar('.');
  }
  while (e--);
}

/** write decimal digits from a long signed int
  \param v number to send
  \param fp number of decimal places to the right of the decimal point
*/
void write_int32_vf(void (*writechar)(uint8_t), int32_t v, uint8_t fp) {
  if (v < 0) {
    writechar('-');
    v = -v;
  }

  write_uint32_vf(writechar, v, fp);
}


#if __SIZEOF_INT__ == 2
#define GET_ARG(T) (va_arg(args, T))
#elif __SIZEOF_INT__ >= 4
#define GET_ARG(T) (va_arg(args, T))
#else
#define GET_ARG(T) (va_arg(args, T))
#endif

void sendf_P(void (*writechar)(uint8_t), PGM_P format_P, ...) {
  //noInterrupts();
  va_list args;
  va_start(args, format_P);

  uint16_t i = 0;
  uint8_t c = 1, j = 0;
  while ((c = pgm_read_byte(&format_P[i++]))) {
    //delayMicroseconds(1);
    if (j) {
      switch (c) {
        case 'd':
          write_int32(writechar, GET_ARG(int32_t));
          j = 0;
          break;
        case 'f':
        case 'q':
          write_int32_vf(writechar, (int32_t)GET_ARG(uint32_t), 3);
          j = 0;
          break;
        case 's':
          write_char(writechar, (char*)GET_ARG(char*));
          j = 0;
          break;
        default:
          writechar(c);
          j = 0;
          break;
      }
    }
    else {
      if (c == '%') {
        j = 4;
      }
      else {
        writechar(c);
      }
    }
  }
  va_end(args);
  //interrupts();
}


