//#pragma once
#ifndef xmyboards_H
  #define myboards_H

  //#include "platform.h"
  //#include "config_pins.h"

  /*
    ============================================================================================
       _WEMOS_CNC_ONLY
    ============================================================================================
  */
  #if defined(BOARD_WEMOSCNC_ONLY_V2)
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



    //#define temp_pin A0
    //#define heater_pin D1
    //#define heater_pin D1
    #define TOOLON HIGH
    #ifndef tool1_pin
    #define tool1_pin D1
    #endif


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
  #else
    #warning No BOARD Defined !
  #endif
#endif
