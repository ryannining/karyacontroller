/*
============================================================================================
    AVR
============================================================================================
*/
#if defined(__AVR__)
#include<arduino.h>

//Known board in boards.h

//#define BOARD_TARANTHOLE
#define BOARD_NANONANO
//#define BOARD_RAMP13
//#define BOARD_SEMEDIY128AU
//#define BOARD_ESP01CNC_V1

#include "boards.h"
#define USE_EEPROM

#else
// for PC no pins
#endif


/*
============================================================================================
    CONFIGURATION 
============================================================================================
*/

#define motortimeout 10000000 // 30 seconds

#define DRIVE_COREXY
//#define DRIVE_COREXZ

#define HOMINGSPEED 100
#define XOFFSET 0
#define YOFFSET 0
#define ZOFFSET 0
#define EOFFSET 0

#define XJERK 25
#define YJERK 25
#define ZJERK 15
#define E0JERK 5

#define XACCELL 300
#define YACCELL 300
#define ZACCELL 500
#define E0ACCELL 100

#define XMOVEACCELL 300
#define YMOVEACCELL 300
#define ZMOVEACCELL 1500
#define ZMOVEACCELL 1500

#define XMAXFEEDRATE 200
#define YMAXFEEDRATE 200
#define ZMAXFEEDRATE 50
#define E0MAXFEEDRATE 6

#define XSTEPPERMM 100
#define YSTEPPERMM 100
#define ZSTEPPERMM 400//420
#define E0STEPPERMM 150//380

#define NUMBUFFER 11
#define XMAX 0
#define YMAX 0
#define ZMAX 168


