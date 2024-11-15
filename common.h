//#pragma once
#ifndef common_H
#define common_H

#include "motion.h"


#define BUFSIZE     256

#define buf_canread(buffer)     ((buffer ## head - buffer ## tail ) & \
                                 (BUFSIZE - 1))

#define buf_pop(buffer, data)   do { \
    data = buffer ## buf[buffer ## tail]; \
    buffer ## tail = (buffer ## tail + 1) & \
                     (BUFSIZE - 1); \
  } while (0)

#define buf_canwrite(buffer)    ((buffer ## tail - buffer ## head - 1) & \
                                 (BUFSIZE - 1))

#define buf_push(buffer, data)  do { \
    buffer ## buf[buffer ## head] = data; \
    buffer ## head = (buffer ## head + 1) & \
                     (BUFSIZE - 1); \
  } while (0)

#define MASK(PIN) (1 << PIN)



//const float powers[] = {1.f, 0.1f, 0.01f,0.001f,0.0001f, 0.00001f, 0.000001f, 0.0000001f, 0.00000001f, 0.000000001f};;
//#define POWERS(e) (powers[e])
//#define POWERS(e) (float)pgm_read_float(&(powers[e]))
const int32_t PROGMEM powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};;
#define POWERS(e) (int32_t)pgm_read_dword(&(powers[e]))
#define DECFLOAT_EXP_MAX 6


#include <Arduino.h>



//#ifdef __ARM__
static bool hasSerial = true;
#ifdef WIFISERVER
extern void wifiwr(uint8_t s);
static void serialwr(uint8_t s) {
  if (hasSerial)Serial.write(s);
  wifiwr(s);
}
#else
static void serialwr(uint8_t s) {
  if (hasSerial)Serial.write(s);
}
#endif
static void serialwr0(uint8_t s) {
  if (hasSerial)Serial.write(s);
}

//#else
//#define serialwr Serial.write
//#endif
#define serialrd(s) s=Serial.read()
#define serialav() Serial.available()
#define serialinit(baud) Serial.begin(baud)


void sendf_P(void (*writechar)(uint8_t), PGM_P format_P, ...);
// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.

//#define xprintf(...) sendf_P(serial_writechar, __VA_ARGS__)
//#define sersendf_P(...) sendf_P(serial_writechar, __VA_ARGS__)

#define zprintf(...)   sendf_P(serialwr, __VA_ARGS__)
#define xprintf(...)   sendf_P(serialwr0, __VA_ARGS__)

#define ff(f) int32_t(1000.f*f)
#define fg(f) int32_t(100.f*f)
#define fi(f) (int32_t)f

#endif
