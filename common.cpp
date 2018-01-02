#include "common.h"
#include  <stdarg.h>

#if defined(__AVR__) || defined(ESP8266)
// functions for sending decimal
#include<arduino.h>

#endif

#define write_uint8(v, w)  write_uint32(v, w)
#define write_int8(v, w)   write_int32(v, w)
#define write_uint16(v, w) write_uint32(v, w)
#define write_int16(v, w)  write_int32(v, w)

// send one character



/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void write_hex4(void (*writechar)(uint8_t), uint8_t v) {
	v &= 0xF;
	if (v < 10)
    writechar('0' + v);
	else
    writechar('A' - 10 + v);
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void write_hex8(void (*writechar)(uint8_t), uint8_t v) {
  write_hex4(writechar, v >> 4);
  write_hex4(writechar, v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void write_hex16(void (*writechar)(uint8_t), uint16_t v) {
  write_hex8(writechar, v >> 8);
  write_hex8(writechar, v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void write_hex32(void (*writechar)(uint8_t), uint32_t v) {
  write_hex16(writechar, v >> 16);
  write_hex16(writechar, v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void write_uint32(void (*writechar)(uint8_t), uint32_t v) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
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

/** write decimal digits from a long unsigned int
\param v number to send
\param fp number of decimal places to the right of the decimal point
*/
void write_uint32_vf(void (*writechar)(uint8_t), uint32_t v, uint8_t fp) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	if (e < fp)
		e = fp;

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
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


#if __SIZEOF_INT__ >= 4
  #define GET_ARG(T) ((T)va_arg(args, int))
#else
  #define GET_ARG(T) ((T)va_arg(args, T))

#endif

void sendf_P(PGM_P format_P, ...) {
  void (*writechar)(uint8_t) =serial_writechar;
	va_list args;
	va_start(args, format_P);

	uint16_t i = 0;
	uint8_t c = 1, j = 0;
	while ((c = pgm_read_byte(&format_P[i++]))) {
		if (j) {
			switch(c) {
				case 'u':
                    write_uint32(writechar, GET_ARG(uint32_t));
					j = 0;
					break;
				case 'd':
                    write_int32(writechar, GET_ARG(int32_t));
					j = 0;
					break;
				case 'c':
                    writechar((uint8_t)GET_ARG(uint32_t));
					j = 0;
					break;
				case 'x':
                      writechar('0');
                      writechar('x');
                      if (j == 1)
                        write_hex8(writechar, (uint8_t)GET_ARG(uint16_t));
                      else if (j == 2)
                        write_hex16(writechar, (uint16_t)GET_ARG(uint16_t));
                      else
                        write_hex32(writechar, GET_ARG(uint32_t));
					j = 0;
					break;
                case 'f':
                          double xx;
                          xx=(double)GET_ARG(double);
                          write_int32_vf(writechar, int(1000*xx), 3);
                          j = 0;
                  break;
				case 'q':
                    write_int32_vf(writechar, GET_ARG(uint32_t), 3);
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
}

