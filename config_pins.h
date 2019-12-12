/*
  ============================================================================================
    AVR
  ============================================================================================
*/
#ifndef CONFIG_PINS
#define CONFIG_PINS
//#define SHARE_EZ
#define NUMBUFFER 20
#include "platform.h"
#define THEISR
#ifdef ISPC
#define LOW false
#define HIGH true
#endif

#define LASERON HIGH
#define SPINDLEON HIGH

#ifndef ISPC
#include<Arduino.h>

//Known board in boards.h
#define xenable -1
#define yenable -1
#define zenable -1
#define e0enable -1

#define ISRTEMP // 120bytes check board.h


#define MAXTEMP 249

// ========== AVR ================================================
#if defined(__AVR__)
#define BOARD_CHCSHIELDV3
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
//#define BOARD_DIY_CNC1
//#define BOARD_SEMEDIY128AU
#define ANALOGSHIFT 0 // 10bit adc
//#define SUBPIXELMAX 0  // multiple axis smoothing / AMASS maximum subpixel
#define BAUDRATE 115200

// ======= STM32F103 ===================================================
#elif defined(__ARM__)
//#define SUBPIXELMAX 6  // multiple axis smoothing / AMASS maximum subpixel
//#define BOARD_NANONANO_STM32
//#define BOARD_ST33DV1_STM32
#define BOARD_ST33DV11_STM32
//#define BOARD_ST33DV1_STM32_3DPLEX
//#define BOARD_ST33DV1_XYYZ_STM32
//#define BOARD_ST33DV1_CNC_STM32
//#define BOARD_STM32F0

#define ANALOGSHIFT 2 // 12bit adc
//#define SUBPIXELMAX 1  // multiple axis smoothing / AMASS maximum subpixel // set 1 to disable but can be adjust using M291 Sxx
//#define EMULATETEMP
#define BAUDRATE 115200*1


// ====== ESP32 ====================================================
#elif defined(ESP32)
#define BOARD_ESP32VN3D
#define THEISR ICACHE_RAM_ATTR
#define SUBPIXELMAX 1  // multiple axis smoothing / AMASS maximum subpixel
#define EMULATETEMP
#define BAUDRATE 115200*1


// ====== ESP8266 ====================================================
#elif defined(ESP8266)
#define MAXTEMP 249
#define SUBPIXELMAX 0  // multiple axis smoothing / AMASS maximum subpixel
#define THEISR ICACHE_RAM_ATTR
#define ANALOGSHIFT 0 // 10bit adc ??
#define BAUDRATE 115200*2
#define NUMBUFFER 35


//#define BOARD_NANONANO_WEMOS
//#define BOARD_WEMOS3D
//#define BOARD_WEMOS3D_COREXY
//#define BOARD_ESPUNO_COREXY
#define BOARD_WEMOS_CNC_XZYY
//#define BOARD_WEMOS3DCOREXY
//#define BOARD_WEMOSCNC
//#define BOARD_MINICNC_ESP01
//#define BOARD_WEMOS_XYY_LASER
//#define BOARD_ESP01CNC_V1
#define EMULATETEMP
#endif

#include "myboards.h"
#define USE_EEPROM

#else
// for PC no pins
#endif


/*
  ============================================================================================
    CONFIGURATION
  ============================================================================================
*/

#ifdef __AVR__
//#define USEDIO // 750bytes this can save almost 20us each bresenham step, is a MUST if not using timer!
#define USETIMER1 // Work in progress // 98 bytes// FLASH SAVING
#define CORESERIAL // smaller footprint 500byte, only AVR
#define SAVE_RESETMOTION  // 1000 bytes code, no reset motion, need EEPROM
#define USE_BACKLASH  // 400bytes code
//#define MESHLEVEL // 4Kb
// ==========================================================

#else
#define MESHLEVEL
#define ARC_SUPPORT // 3kb
#define USE_BACKLASH  // 400bytes code
#define USETIMER1 // Work in progress // 98 bytes// FLASH SAVING
//#define LCDDISPLAY 0x3F // more than 2.5K , simple oled controller
//#define CHANGEFILAMENT //580byte
#define HARDSTOP // allow to stop in the middle of movement, and still keep the current position, great for CNC
#define WIFISERVER
//#define TOUCHSERVER
#endif
// ==========================================================

//#define ACT_KEY

//
#ifndef temp_pin
#define EMULATETEMP
#endif


#ifdef EMULATETEMP
#undef temp_pin
#endif


//#define MYLASER

#ifdef MYLASER
//#undef WIFISERVER
#undef heater_pin
#define LASERON LOW
#define NUMBUFFER 50
#endif

#define LASERON LOW
//#define NUMBUFFER 50

//#define USE_EEPROM

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


#ifndef ISPC
#define SUBPIXELMAX 0  // multiple axis smoothing / AMASS maximum subpixel
#else
#define SUBPIXELMAX 4
#endif


//#undef SDCARD_CS
#ifdef SDCARD_CS
#define USE_SDCARD
#endif


// ESP8266
#ifdef SHARE_EZ
#warning Share E and Z direction
#endif


#ifndef ESP8266
#undef WIFISERVER

#endif

#ifndef __AVR__

// not implemented on non AVR
#undef USEDIO
#undef ISRTEMP
#undef CORESERIAL
//#undef USE_EEPROM
//#undef LCDDISPLAY
//#undef USETIMER1

#endif

#ifdef ISPC
// not implemented on PC
#undef USETIMER1
#undef LASERMODE
#undef SAVE_RESETMOTION
#undef ACT_KEY

//#define DRIVE_XYYZ  // dual Y individual homing
//#define DRIVE_COREXY
//#define DRIVE_COREXZ

//#define DRIVE_DELTA
//#define DRIVE_DELTASIAN

#endif


//#define motortimeout 10000000 // 10 seconds



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
#define HOMINGSPEED 30
#define XOFFSET 0
#define YOFFSET 0
#define ZOFFSET 0
#define EOFFSET 0


#ifdef BOARD_WEMOS3D_COREXY
  #define XYJERK 600
  #define XYCORNER 45
  #define XACCELL 1600
#else
  #define XYJERK 800
  #define XYCORNER 235
  #define XACCELL 1200
#endif

#ifdef BOARD_WEMOS3D_COREXY
  #define XMAXFEEDRATE 100
  #define YMAXFEEDRATE 100
  #define ZMAXFEEDRATE 30
  #define E0MAXFEEDRATE 25
#else
  #define XMAXFEEDRATE 100
  #define YMAXFEEDRATE 100
  #define ZMAXFEEDRATE 100
  #define E0MAXFEEDRATE 100
#endif

#ifdef BOARD_WEMOS3D_COREXY
  #define XSTEPPERMM 100.5//50//105.090//50//131//178
  #define YSTEPPERMM 100.5////105.090//50//175//125
  #define ZSTEPPERMM 243.75//2300//80//1020//1020 //420
  #define E0STEPPERMM 152//92//340//380
#else
  #define XSTEPPERMM 150//50//105.090//50//131//178
  #define YSTEPPERMM 150////105.090//50//175//125
  #define ZSTEPPERMM 150//2300//80//1020//1020 //420
  #define E0STEPPERMM 100//92//340//380
#endif

#ifndef NUMBUFFER
  #define NUMBUFFER 20
#endif
#ifdef BOARD_WEMOS3D_COREXY
  #define XMAX 0
  #define YMAX 0
  #define ZMAX 159
#else
  #define XMAX 1
  #define YMAX 1
  #define ZMAX 1
#endif

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

#ifndef KBOX_KEY1_R

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
#endif // 

#endif
#endif // config_pins
