/*
============================================================================================
    AVR
============================================================================================
*/
#include "motion.h"
#ifndef ISPC
#include<arduino.h>

//Known board in boards.h

//#define BOARD_CHCSHIELDV3
//#define BOARD_TARANTHOLE
//#define BOARD_SEMEDIYNANO
//#define BOARD_NANONANO
//#define BOARD_GEN7
//#define BOARD_RAMP13
#define BOARD_RAMP13_3DPLEX
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

//#define DRIVE_COREXY
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

#define XACCELL 1300
#define YACCELL 1300
#define ZACCELL 500
#define E0ACCELL 100

#define XMOVEACCELL 300
#define YMOVEACCELL 300
#define ZMOVEACCELL 1500
#define ZMOVEACCELL 1500

#define XMAXFEEDRATE 100
#define YMAXFEEDRATE 100
#define ZMAXFEEDRATE 50
#define E0MAXFEEDRATE 6

#define XSTEPPERMM 100
#define YSTEPPERMM 100
#define ZSTEPPERMM 400//420
#define E0STEPPERMM 150//380

#define NUMBUFFER 13
#define XMAX 0
#define YMAX 0
#define ZMAX 168

#define USE_BACKLASH

#define MOTOR_X_BACKLASH 0  // MOTOR 0 = X, 1= Y 2=Z 3=E
#define MOTOR_Y_BACKLASH 0
#define MOTOR_Z_BACKLASH 0
#define MOTOR_E_BACKLASH 0

//#define AUTO_MOTOR_Z_OFF


#define MOTOR_0_DIR 1 // 1: normal -1:inverted
#define MOTOR_1_DIR 1 // 1: normal -1:inverted
#define MOTOR_2_DIR 1 // 1: normal -1:inverted
#define MOTOR_3_DIR 1 // 1: normal -1:inverted


//#define INVERTENDSTOP // uncomment for normally open

#define ENDSTOP_MOVE 2.5   //2mm move back after endstop hit, warning, must 
