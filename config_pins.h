//#pragma once

#ifndef configpin_H
#define configpin_H



#define NUMBUFFER 20




#define THEISR

#define LASERON HIGH
#define SPINDLEON HIGH

#include<Arduino.h>

//Known board in boards.h
#define xenable -1
#define yenable -1
#define zenable -1
#define e0enable -1

#define ISRTEMP // 120bytes check board.h
#define INDEX "/3d.html"

#define MAXTEMP 249


#if defined(ESP32)
	#warning CPU ESP32 
	#define BOARD_ESP32VN3D
	#define THEISR IRAM_ATTR
	#define EMULATETEMP
	#define BAUDRATE 115200*2
	#define NUMBUFFER 30


// ====== ESP8266 ====================================================
#elif defined(ESP8266)
	#warning CPU ESP8266
	#define MAXTEMP 249
	#define THEISR IRAM_ATTR
	#define ANALOGSHIFT 0 // 10bit adc ??
	#define BAUDRATE 115200*2
	#define NUMBUFFER 30

	//#define BOARD_NANONANO_WEMOS
	//#define BOARD_WEMOS3D
	//#define BOARD_ESPUNO_COREXY
	//#define BOARD_WEMOS3D_COREXY
	//#define BOARD_WEMOS_CNC_XZYY
	//#define BOARD_WEMOS3DCOREXY
	//#define BOARD_WEMOSCNC
	
	#define MYLASER
	//#define LASERMINI
	//#define LASERBIG
	//#define LASERMERAH
	#define LCLASER


	#ifndef MYLASER
		#define MK4CNC
		// subset cnc, if not dfault to mini cnc
		//#define CNCBIG
		//#define M115
		#define LCHI1224
		//#define LC_PLASMA
	#endif
	
	#define EMULATETEMP

	#ifdef MYLASER
		#undef MK4CNC
		#define IR_OLED_MENU
		#undef BOARD_WEMOS3D_COREXY
		#undef BOARD_WEMOS_CNC_XZYY
		//#define BOARD_WEMOSCNC
		#ifdef LASERBIG
			#define BOARD_WEMOSCNC_ONLY
		#else
			#define BOARD_WEMOSCNC_ONLY_V2
			#define COPY_Y_TO_Z
		#endif
	#endif

	//#define OLD_BLUE_BOARD

	#ifdef MK4CNC
		#undef laser_pin
		#define IR_OLED_MENU
		#undef BOARD_WEMOS3D_COREXY
		#undef BOARD_WEMOS_CNC_XZYY
		#ifdef OLD_BLUE_BOARD
			#define BOARD_WEMOSCNC_ONLY
		#else
			#define BOARD_WEMOSCNC_ONLY_V2
		#endif
	#endif

	#ifdef LC_PLASMA

		#define BOARD_WEMOSCNC_ONLY_V2
		#define PLASMA_MODE
		//#define limit_pin D2 // for Z probe
		#define ANALOG_THC
	#endif

#endif

//#define BOARD_WEMOSCNC_ONLY
#include "myboards.h"

#define USE_EEPROM

// custom
//#define spindle_pin D1


#ifdef MK4CNC
	#undef laser_pin
#endif

#ifdef MYLASER
	#undef zstep
	#undef zdirection
	#undef spindle_pin	
#endif

/*
  ============================================================================================
    CONFIGURATION
  ============================================================================================
*/

//#define PLOTTING
//#define MESHLEVEL
#define ARC_SUPPORT // 3kb
#define USE_BACKLASH  // 400bytes code
//#define CHANGEFILAMENT //580byte
#define HARDSTOP // allow to stop in the middle of movement, and still keep the current position, great for CNC
//#define WIFISERVER
//#define TOUCHSERVER  // Karyacnc WEB SERVER ?

// enable LCD on this machine
#if defined(M115) || defined(LASERMINI) || defined(LASERBIG)|| defined(LCLASER)
	#define IR_OLED_MENU
#endif

#ifdef ESP8266
	#define LCD_SDA TX
	#define LCD_SCL RX
	#define LCD_CMD D1
	#define LCD_CS RX

	#define USEOTA
	//#define TCPSERVER // Old style for Repetier ??
	#define WEBSOCKSERVER
	#define WIFISERVER

#endif

#if defined(ESP32)
	#define LCD_SDA TX
	#define LCD_SCL RX
	#define LCD_CMD D1
	#define LCD_CS RX

	#define USEOTA
	//#define TCPSERVER
	#define WIFISERVER
	#define WEBSOCKSERVER  // need wifiserver
#endif

#ifdef IR_OLED_MENU
	// pick LCD model
	// to debug
	//#undef IR_OLED_MENU
	//FOR quick dev test 1661
	#undef LCD_OLED_SSD
	#undef LCD_OLED
	#undef LCD_UC1609
	#undef LCD_NK1661
	#undef LCD_NK1202
	#define LCD_NK1661
	#define HAS_CS LCD_CS
	#define IR_KEY LCD_SDA //share remote data pin with LCD SDA pin

#endif



#ifndef temp_pin
	#define EMULATETEMP
#endif


#ifdef EMULATETEMP
	#undef temp_pin
#endif


//#define POMPA
///* DEBUG
#ifdef POMPA
#undef ANALOG_THC
#undef IR_KEY
//#define IR_KEY D2
#undef IR_OLED_MENU
//*/
#endif



#define LASERON LOW // depends on laser machine

#ifdef MYLASER
	#define LASERON HIGH
	#define laser_pin D1
	#warning "is LASER ?"
	#undef spindle_pin
	#define BAUDRATE 115200*2
	#define NUMBUFFER 65

	//LOW

	#ifdef LASERMINI
		#define LASERON HIGH
		#define laser_pin D1
	#endif

	#ifdef LASERBIG
		#define LASERON HIGH
		#define laser_pin D1
		//#define HAS_CS D2
	#endif

	#ifdef LCLASER
		#define LASERON HIGH
		#define laser_pin D1
		//#define HAS_CS D2
	#endif

	#undef heater_pin
	#undef temp_pin
#endif


//#define USE_EEPROM

#ifdef powerpin
	#define POWERFAILURE
#endif

// lets assume if not laser_pin not defined use the heater_pin

#ifdef laser_pin
	#define LASERMODE
#elif defined(heater_pin)

// to make sure all board can be user for laser engraving
//#define laser_pin heater_pin
//#define LASERMODE
#endif



// ESP8266
#ifdef SHARE_EZ
	#warning Share E and Z direction
#endif


#if defined(__AVR__) || defined (__ARM__)
	#warning Doesnot support ARM/AVR
#endif

//#define motortimeout 10000000 // 10 seconds


#define TOWER_X_ANGLE_DEG        210
#define TOWER_Y_ANGLE_DEG        330
#define TOWER_Z_ANGLE_DEG        90
#define DELTA_DIAGONAL_ROD 180
#define DELTA_RADIUS 85



#ifdef BOARD_WEMOS3D_COREXY

	#define XYCORNER 45
	#define XACCELL 1600
	#define XMAXFEEDRATE 400
	#define YMAXFEEDRATE 400
	#define ZMAXFEEDRATE 30
	#define E0MAXFEEDRATE 25
	#define XSTEPPERMM 100.5//50//105.090//50//131//178
	#define YSTEPPERMM 100.5////105.090//50//175//125
	#define ZSTEPPERMM 243.75//2300//80//1020//1020 //420
	#define E0STEPPERMM 152//92//340//380
	#define MOTOR_X_BACKLASH 0  // MOTOR 0 = X, 1= Y 2=Z 3=E
	#define MOTOR_Y_BACKLASH 0
	#define MOTOR_Z_BACKLASH 0
	#define MOTOR_E_BACKLASH 0
#endif


#ifdef M115
	// just for 115
	#define XSTEPPERMM -426.577
	#define YSTEPPERMM 427.354
	#define ZSTEPPERMM -400.000
	// TRiAL RPM counter
	// #define RPM_COUNTER D1
	//#undef AC_SPINDLE // using DC
#endif

#ifdef CNCBIG
	// just for new cnc BIG with custom gear
	#define XSTEPPERMM 255.965 // gear 10:47 htd5m drv8825
	#define YSTEPPERMM 401.191 // gear 10:47 htd5m drv8825
	#define ZSTEPPERMM 400.000
#endif
#ifdef LC_PLASMA
	// just for new cnc BIG with custom gear
	#define spindle_pin D1
	#undef laser_pin
	#define XSTEPPERMM 262.049 // gear 10:47 htd5m drv8825
	#define YSTEPPERMM 188.32 // gear 10:47 htd5m drv8825
	#define ZSTEPPERMM 400.000
#endif

#ifdef LCHI1224
	// just for new cnc BIG with custom gear
	#ifdef OLDLCHI1224
		#define XSTEPPERMM -255.875 // gear 10:47 htd5m drv8825
		#define YSTEPPERMM -255.899 // gear 10:47 htd5m drv8825
		#define ZSTEPPERMM 800
	#else
		#define XSTEPPERMM -240.280 // gear 10:47 htd5m drv8825
		#define YSTEPPERMM -240.280 // gear 10:47 htd5m drv8825
		#define ZSTEPPERMM 800
	#endif
	

	#define XYCORNER 30
	#define XACCELL 400
	#define XMAXFEEDRATE 60
	#define YMAXFEEDRATE 60
	#define ZMAXFEEDRATE 20
#endif

#ifdef MYLASER

	#define XYCORNER 25
	#define XACCELL 4000
	#define XMAXFEEDRATE 230
	#define YMAXFEEDRATE 170
	#define ZMAXFEEDRATE 200
	#define E0MAXFEEDRATE 100
	#define ZSTEPPERMM 10
	#define E0STEPPERMM 10

	#define XSTEPPERMM 78.740
	#define YSTEPPERMM 314.136

	#ifdef LCLASER

		#define XSTEPPERMM -255.875 // gear 10:47 htd5m drv8825
		#define YSTEPPERMM -255.899 // gear 10:47 htd5m drv8825
		#define ZSTEPPERMM 804.580

		#define XYCORNER 25
		#define XACCELL 2000
		#define XMAXFEEDRATE 300
		#define YMAXFEEDRATE 60
		#define ZMAXFEEDRATE 50
	#endif


	#ifdef LASERMERAH
		#define XSTEPPERMM -157.480 // gear 10:47 htd5m drv8825
		#define YSTEPPERMM -157.068 // gear 10:47 htd5m drv8825
		#define ZSTEPPERMM 50.0

		#define XYCORNER 25
		#define XACCELL 2000
		#define XMAXFEEDRATE 170
		#define YMAXFEEDRATE 60
		#define ZMAXFEEDRATE 50
		#define laser_pin D8
	#endif



	#ifdef LASERBIG
		#define XYCORNER 15
		#define XACCELL 1200
		#define XMAXFEEDRATE 130
		#define YMAXFEEDRATE 40
		#define XSTEPPERMM 200
		#define YSTEPPERMM -327.631
		#define xdirection D6
		#define xstep D0
		#define ydirection D7
		#define ystep D3
		#define zdirection D5
		#define zstep D4
	#endif

	#ifdef LASERMINI
		#define XSTEPPERMM 78.82
		#define YSTEPPERMM 78.82
		#define ZSTEPPERMM 10
		#define xdirection D6
		#define xstep D0
		#define ydirection D7
		#define ystep D3
		#define zdirection D5
		#define zstep D4
	#endif

#endif

#ifndef XSTEPPERMM
	#define XSTEPPERMM 100//50//105.090//50//131//178
	#define YSTEPPERMM 100////105.090//50//175//125
	#define ZSTEPPERMM 100//2300//80//1020//1020 //420
	#define E0STEPPERMM 100//92//340//380
#endif

#ifndef XMAXFEEDRATE
	#define XMAXFEEDRATE 100
	#define YMAXFEEDRATE 100
	#define ZMAXFEEDRATE 100
	#define E0MAXFEEDRATE 100
#endif

#ifndef XACCELL

	#define XYCORNER 35
	#define XACCELL 200
#endif

#ifndef MOTOR_X_BACKLASH
	#define MOTOR_X_BACKLASH 0  // MOTOR 0 = X, 1= Y 2=Z 3=E
	#define MOTOR_Y_BACKLASH 0
	#define MOTOR_Z_BACKLASH 0
	#define MOTOR_E_BACKLASH 0
#endif

#ifndef XOFFSET
#define XOFFSET 0
#define YOFFSET 0
#define ZOFFSET 0
#define EOFFSET 0
#endif

#ifndef LSCALE
#define LSCALE 1
#endif

// Motion configuration
#ifndef CHECKENDSTOP_EVERY
#define CHECKENDSTOP_EVERY 0.05  // mm this translate to 200step if step/mm is 4000, must lower than 255 (byte size)
#endif

#ifndef HOMINGSPEED
#define HOMINGSPEED 30
#endif


#ifndef NUMBUFFER
	#define NUMBUFFER 20
#endif

#ifdef BOARD_WEMOS3D_COREXY
	#define XMAX 0
	#define YMAX 0
	#define ZMAX 159
#else
	#define XMAX 0
	#define YMAX 0
	#define ZMAX 0
#endif



//#define AUTO_MOTOR_Z_OFF



//#define INVERTENDSTOP // uncomment for normally open

#define ENDSTOP_MOVE 3   //2mm move back after endstop hit, warning, must
#define HOMING_MOVE 2000

#include "platform.h"
#include "motors.h"

#endif
