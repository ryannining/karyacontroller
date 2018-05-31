/*
  ============================================================================================
    AVR
  ============================================================================================
*/

#include "motion.h"
#ifndef ISPC
#include<arduino.h>

//Known board in boards.h
#define xenable -1
#define yenable -1
#define zenable -1
#define e0enable -1

#define ISRTEMP // 120bytes check board.h
#define MOTOR_0_DIR -1 // 1: normal -1:inverted
#define MOTOR_1_DIR -1 // 1: normal -1:inverted
#define MOTOR_2_DIR -1 // 1: normal -1:inverted
#define MOTOR_3_DIR 1 // 1: normal -1:inverted
#define THEISR

// ========== AVR ================================================
#if defined(__AVR__)
//#define BOARD_CHCSHIELDV3
//#define BOARD_TARANTHOLE
//#define BOARD_SEMEDIYNANO
//#define BOARD_NANONANO
//#define BOARD_NANONANO_DELTA
//#define BOARD_NANONANO_DELTA_NOSD
//#define BOARD_NANONANO_SDCARD
//#define BOARD_GEN7
//#define BOARD_RAMP13
//#define BOARD_RAMP13_DELTA
//#define BOARD_RAMP13_3DPLEX
//#define BOARD_NANO_3DPLEX
//#define BOARD_DIY_4XI
#define BOARD_DIY_CNC1
//#define BOARD_SEMEDIY128AU
#define ANALOGSHIFT 0 // 10bit adc
#define SUBPIXELMAX 0  // multiple axis smoothing / AMASS maximum subpixel
// ======= STM32F103 ===================================================
#elif defined(__ARM__)
//#define SUBPIXELMAX 6  // multiple axis smoothing / AMASS maximum subpixel
//#define BOARD_NANONANO_STM32
//#define BOARD_ST33DV1_STM32
//#define BOARD_ST33DV1_STM32_3DPLEX
#define BOARD_ST33DV1_XYYZ_STM32
//#define BOARD_ST33DV1_CNC_STM32
#define ANALOGSHIFT 2 // 12bit adc
#define SUBPIXELMAX 6  // multiple axis smoothing / AMASS maximum subpixel
// ====== ESP32 ====================================================
#elif defined(ESP32)
#define BOARD_ESP32VN3D
#define THEISR ICACHE_RAM_ATTR 
//#define SUBPIXELMAX 6  // multiple axis smoothing / AMASS maximum subpixel
// ====== ESP8266 ====================================================
#elif defined(ESP8266)
#define SUBPIXELMAX 6  // multiple axis smoothing / AMASS maximum subpixel
#define THEISR ICACHE_RAM_ATTR 
#define ANALOGSHIFT 0 // 10bit adc ??


//#define BOARD_NANONANO_WEMOS
//#define BOARD_WEMOS3D
//#define BOARD_WEMOSCNC
//#define BOARD_MINICNC_ESP01
#define BOARD_WEMOS_XYY_LASER
//#define BOARD_ESP01CNC_V1
#endif

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

//#define ARC_SUPPORT // 3kb

#define USEDIO // 750bytes this can save almost 20us each bresenham step, is a MUST if not using timer!
#define USE_BACKLASH  // 400bytes code
#define USETIMER1 // Work in progress // 98 bytes// FLASH SAVING
//#define SAVE_RESETMOTION  // 1000 bytes code, no reset motion, need EEPROM
//#define LCDDISPLAY 0x3F // more than 2.5K , simple oled controller
#define CORESERIAL // smaller footprint 500byte, only AVR
#define CHANGEFILAMENT //580byte
#define HARDSTOP // allow to stop in the middle of movement, and still keep the current position, great for CNC
#define WIFISERVER
#define TELEGRAM
// ==========================================================

//#define INTERPOLATEDELAY  // slower 4-8us

#ifdef powerpin
#define POWERFAILURE
#endif

// lets assume if not laser_pin not defined use the heater_pin 

#ifdef laser_pin
#define LASERMODE
#elif defined(heater_pin)

// to make sure all board can be user for laser engraving
#define laser_pin heater_pin
#define LASERMODE
#endif


#define UPDATE_F_EVERY 2000 //us = 250 tick/sec acceleration change
#ifndef ISPC
//#define SUBPIXELMAX 6  // multiple axis smoothing / AMASS maximum subpixel
#else
//#define SUBPIXELMAX 4
#endif


//#undef SDCARD_CS
#ifdef SDCARD_CS
#define USE_SDCARD
#endif


// ESP8266
#if defined(ESP8266) && !defined(USE_SHIFTREG)
#define SHARE_EZ
#endif


#ifndef ESP8266
#undef WIFISERVER

#endif

#ifndef __AVR__

// not implemented on non AVR
#undef USEDIO
#undef ISRTEMP
#undef CORESERIAL
//#undef LCDDISPLAY
//#undef USETIMER1

#endif

#ifdef ISPC
// not implemented on PC
#undef USETIMER1
#undef LASERMODE
#undef SAVE_RESETMOTION
#endif


//#define motortimeout 10000000 // 10 seconds

//#define DRIVE_XYYZ  // dual Y individual homing
//#define DRIVE_COREXY
//#define DRIVE_COREXZ

//#define DRIVE_DELTA
//#define DRIVE_DELTASIAN


#ifdef DRIVE_DELTA
#define NONLINEAR
#endif
#ifdef DRIVE_DELTASIAN
#define NONLINEAR
#endif


#define TOWER_X_ANGLE_DEG        210
#define TOWER_Y_ANGLE_DEG        330
#define TOWER_Z_ANGLE_DEG        90
#define DELTA_DIAGONAL_ROD 180
#define DELTA_RADIUS 85

// Motion configuration
#define CHECKENDSTOP_EVERY 0.05  // mm this translate to 200step if step/mm is 4000, must lower than 255 (byte size)
#define HOMINGSPEED 60
#define XOFFSET 0
#define YOFFSET 0
#define ZOFFSET 0
#define EOFFSET 0

#define XYJERK 22
#define XACCELL 40
#define XMOVEACCELL 40

#define XMAXFEEDRATE 240
#define YMAXFEEDRATE 240
#define ZMAXFEEDRATE 240
#define E0MAXFEEDRATE 10

#define XSTEPPERMM 10//131//178
#define YSTEPPERMM 10//175//125
#define ZSTEPPERMM 10//1020//1020 //420
#define E0STEPPERMM 10//340//380

#ifndef NUMBUFFER
#define NUMBUFFER 20
#endif

#define XMAX 1200
#define YMAX 1800
#define ZMAX 20

#define MOTOR_X_BACKLASH 0  // MOTOR 0 = X, 1= Y 2=Z 3=E
#define MOTOR_Y_BACKLASH 0
#define MOTOR_Z_BACKLASH 0
#define MOTOR_E_BACKLASH 0

//#define AUTO_MOTOR_Z_OFF



//#define INVERTENDSTOP // uncomment for normally open

#define ENDSTOP_MOVE 3   //2mm move back after endstop hit, warning, must
#define HOMING_MOVE 2000

// KontrolBox a series resistor with switch to a analog PIN
// MCU only
#ifndef ISPC


/*
    MACROS for KBOXKontroller

*/


#define KBOX_KEY_CHECK(k)   case KBOX_KEY##k##_R : lkey = k;kdl=500;break;
//#define KBOX_SHOW_VALUE
#define KBOX_KEY1_R 0 ... 10
#define KBOX_KEY2_R 500 ... 530
#define KBOX_KEY3_R 670 ... 695
#define KBOX_KEY4_R 750 ... 780

#define KBOX_DO_CHECK  KBOX_KEY_CHECK(1) KBOX_KEY_CHECK(2) KBOX_KEY_CHECK(3) KBOX_KEY_CHECK(4)


#ifdef KBOX_PIN
#define KBOX_KEY4_ACTION zprintf(PSTR("HOMING\n"));homing();
#define KBOX_KEY3_ACTION zprintf(PSTR("HEATING\n"));set_temp(190);
#define KBOX_KEY2_ACTION if (sdcardok) {sdcardok = sdcardok == 1 ? 2 : 1;zprintf(PSTR("SD\n"));} else demoSD();
#define KBOX_KEY1_ACTION RUNNING=0;sdcardok=0;zprintf(PSTR("STOP\n"));power_off();

#define KBOX_KEY_ACT(k)   case k: zprintf(PSTR("Act %d\n"),k); KBOX_KEY##k##_ACTION  ;break;
#define KBOX_DO_ACT  KBOX_KEY_ACT(1) KBOX_KEY_ACT(2) KBOX_KEY_ACT(3) KBOX_KEY_ACT(4)
#else // no controller
//#define KBOX_DO_ACT
#endif

#endif
