//#pragma once
#ifndef xmyboards_H
#define myboards_H

//#include "platform.h"
//#include "config_pins.h"


#if defined(BOARD_ESP01CNC_V1)

#define xenable D1
#define xdirection D2
#define xstep D3
#define yenable D1
#define ydirection D2
#define ystep D3
#define zenable D1
#define zdirection D2
#define zstep D3

#define limit_pin D5

#define temp_pin A0
#define heater_pin D6
/*
  ============================================================================================
     NANONANO_WEMOS
  ============================================================================================
*/
#elif defined(BOARD_NANONANO_WEMOS)

// shift register for all motor step and direction (8 pin)
#define USE_SHIFTREG
#define pinclock D5
#define pinlatch D0
#define pindata D2

// Implemented shift register pin for motors
#define xdirection 6
#define xstep 7
#define ydirection 4
#define ystep 5
#define zdirection 3
#define zstep 2
#define e0direction 0
#define e0step 1


#define xenable D3
#define yenable D3
#define zenable D3
#define e0enable D3

#define limit_pin D1

#define temp_pin A0
#define heater_pin D4
#define fan_pin D4

#define INVERTENDSTOP
//#define SDCARD_CS D8
#define NUMBUFFER 20
/*
  ============================================================================================
     _WEMOS3D
  ============================================================================================
*/
#elif defined(BOARD_WEMOS3D)

#define xdirection D5
#define xstep D4
#define ydirection D7
#define ystep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection D6
#define zstep D0
#define e0direction D6
#define e0step D2

#define limit_pin D8

#define temp_pin A0
#define heater_pin D1

//#define INVERTENDSTOP
#define NUMBUFFER 30
#define SHARE_EZ

/*
  ============================================================================================
     _WEMOS3D
  ============================================================================================
*/
#elif defined(BOARD_WEMOS_CNC_XZYY)
#define INDEX "index.html"

// motor will be X->X board  Z->Y board Y1->Zboard Y2->EBoard

#define xdirection D5
#define xstep D4
#define zdirection D7
#define zstep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define ydirection D6
#define ystep D0

#define e0direction D6
#define e0step D2

#define limit_pin D8

#define temp_pin A0
#define heater_pin D1

//#define INVERTENDSTOP
#define NUMBUFFER 20
#define DRIVE_XZY2
/*
  ============================================================================================
     _WEMOS3D
  ============================================================================================
*/
#elif defined(BOARD_WEMOS3D_COREXY)

#define xdirection D5
#define xstep D4
#define ydirection D7
#define ystep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection D6
#define zstep D0
#define e0direction D6
#define e0step D2

#define limit_pin D8

//#define temp_pin A0
#define heater_pin D1

//#define INVERTENDSTOP
#define NUMBUFFER 20
#define DRIVE_COREXY
#define SHARE_EZ

/*
  ============================================================================================
     _WEMOS3D
  ============================================================================================
*/
#elif defined(BOARD_ESPUNO_COREXY)

#define xdirection D5
#define xstep D2
#define ydirection D6
#define ystep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection D7
#define zstep D4
//#define e0direction D6
//#define e0step D2

#define limit_pin D8

//#define temp_pin A0
#define heater_pin D1

//#define INVERTENDSTOP
#define NUMBUFFER 50
#define DRIVE_COREXY
//#define SHARE_EZ

/*
  ============================================================================================
     _WEMOS3D
  ============================================================================================
*/
#elif defined(BOARD_WEMOSCNC)

#define xdirection D5
#define xstep D4
#define ydirection D7
#define ystep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection D6
#define zstep D0
//#define e0direction D6
//#define e0step D2

#define limit_pin D8

//#define temp_pin A0
//#define heater_pin D1
//#define heater_pin D1
#define laser_pin D1
#define spindle_pin D8
//#define INVERTENDSTOP
#define NUMBUFFER 20

//#define IR_KEY D2
//#define IR_OLED_MENU 0x3c, RX, TX
/*
  ============================================================================================
     _WEMOS_CNC_ONLY
  ============================================================================================
*/
#elif defined(BOARD_WEMOSCNC_ONLY)
#define INDEX "/cnc.html"
#define xdirection D6
#define xstep D0
#define ydirection D7
#define ystep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection D5
#define zstep D4
//#define e0direction D6
//#define e0step D2

#define limit_pin D8

//#define temp_pin A0
//#define heater_pin D1
//#define heater_pin D1
//#define laser_pin D1
#define spindle_pin D8
#define AC_SPINDLE // 
//#define INVERTENDSTOP
#define NUMBUFFER 20

//#define IR_KEY D2
#define LCD_OLED
#define IR_OLED_MENU 0x3c, RX, TX
//#define IR_OLED_MENU 0x3c, RX, D1

#define XSTEPPERMM 229.335//50//105.090//50//131//178
#define YSTEPPERMM 194.053////105.090//50//175//125
#define ZSTEPPERMM 3072//2300//80//1020//1020 //420
#define E0STEPPERMM 100//92//340//380

#define XYCORNER 15
#define XACCELL 150
#define XMAXFEEDRATE 30
#define YMAXFEEDRATE 30
#define ZMAXFEEDRATE 4
#define E0MAXFEEDRATE 4
#define MOTOR_X_BACKLASH 0200  // 0.2 // MOTOR 0 = X, 1= Y 2=Z 3=E
#define MOTOR_Y_BACKLASH 0200
#define MOTOR_Z_BACKLASH 0
#define MOTOR_E_BACKLASH 0
#define LSCALE 1 // Spindle power calibration
/*
  ============================================================================================
     _WEMOS_CNC_ONLY
  ============================================================================================
*/
#elif defined(BOARD_WEMOSCNC_ONLY_V2)
#define INDEX "/cnc.html"
#define xdirection D5
#define xstep D6

#define zdirection D0
#define zstep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define ydirection D7
#define ystep D4
//#define e0direction D6
//#define e0step D2

#define limit_pin D8

//#define temp_pin A0
//#define heater_pin D1
//#define heater_pin D1
#define LASERON HIGH
#define laser_pin D1
#define spindle_pin D8
#define AC_SPINDLE // 
//#define INVERTENDSTOP
#define NUMBUFFER 20

//#define IR_KEY D2
#define LCD_OLED
#define IR_OLED_MENU 0x3c, RX, TX
//#define IR_OLED_MENU 0x3c, RX, D1

#define XSTEPPERMM 229.335//50//105.090//50//131//178
#define YSTEPPERMM 194.053////105.090//50//175//125
#define ZSTEPPERMM 3072//2300//80//1020//1020 //420
#define E0STEPPERMM 100//92//340//380

#define XYCORNER 15
#define XACCELL 150
#define XMAXFEEDRATE 30
#define YMAXFEEDRATE 30
#define ZMAXFEEDRATE 4
#define E0MAXFEEDRATE 4
#define MOTOR_X_BACKLASH 0200  // 0.2 // MOTOR 0 = X, 1= Y 2=Z 3=E
#define MOTOR_Y_BACKLASH 0200
#define MOTOR_Z_BACKLASH 0
#define MOTOR_E_BACKLASH 0
#define LSCALE 1 // Spindle power calibration
/*
  ============================================================================================
     _WEMOSLASER
  ============================================================================================
*/
#elif defined(BOARD_WEMOS_XYY_LASER)

#define xdirection D5
#define xstep D4
#define ydirection D7
#define ystep D3

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection D6
#define zstep D0
#define e0direction D6
#define e0step D2

#define limit_pin D8

//#define temp_pin A0
//#define heater_pin D1
#define laser_pin D1

//#define INVERTENDSTOP
#define DRIVE_XYYZ
#define NUMBUFFER 20
/*
  ============================================================================================
     _ESP32VN3D
  ============================================================================================
*/
#elif defined(BOARD_ESP32VN3D)

#define xdirection 36
#define xstep 39
#define ydirection 34
#define ystep 35

// z and e have same direction pin, we think that normally slicer never move z and e together.. we hope we right :D
#define zdirection 32
#define zstep 33
#define e0direction 25
#define e0step 26

#define limit_pin 27

//#define temp_pin 0
#define heater_pin 14
#define laser_pin 14

#define INVERTENDSTOP
#define NUMBUFFER 50
/*
  ============================================================================================
     NANONANO_WEMOS
  ============================================================================================
*/
#elif defined(BOARD_MINICNC_ESP01)

// shift register for all motor step and direction (8 pin)
#define USE_SHIFTREG
#define RX 3
#define TX 1
#define D0 16
#define D1 5

#define pinclock TX
#define pinlatch D0
#define pindata D1
#define limit_pin RX


// Implemented shift register pin for motors
#define xdirection 1
#define xstep 2
#define ydirection 6
#define ystep 5
#define zdirection 4
#define zstep 7
//#define e0direction 0
//#define e0step 1
#define laser_pin 0
//#define heater_pin 0

#define xenable 3
#define yenable 3
#define zenable 3
#define e0enable 3


#define ENABLEWIFI 10
#define DISABLESERIAL
#define INVERTENDSTOP
#define NUMBUFFER 20
#else
#warning No BOARD Defined !
#endif
#endif
