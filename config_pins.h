#pragma once

#ifndef configpin_H
#define configpin_H



#define NUMBUFFER 20


#define THEISR 


#include<Arduino.h>

//Known board in boards.h
#define xenable -1
#define yenable -1
#define zenable -1
#define e0enable -1

#define ISRTEMP // 120bytes check board.h
#define INDEX "/3d.html"

#define MAXTEMP 249

#define IR_KEY 1
//#define output_enable

#if defined(ESP32)
	#warning CPU ESP32 

	//#define THEISR IRAM_ATTR

	#define BAUDRATE 115200*2
	//#define NUMBUFFER 30
	#define ANALOG_THC
	#define PREMIUMFEATURE
	#define DS18B20


// ====== ESP8266 ====================================================
#elif defined(ESP8266)
	#warning CPU ESP8266

	//#define THEISR IRAM_ATTR
	#define ANALOGSHIFT 0 // 10bit adc ??
	#define BAUDRATE 115200*2
	//#define NUMBUFFER 30


	#define PREMIUMFEATURE

	//#define LC_PLASMA

	// using digital potensiometer to adjust the IC
	// use TX pin ( oled data ) as direction, and use spindle_pin as steps	
	// #define pinX9C TX 

	//#define EMULATETEMP

	#define DS18B20

		
	#define ANALOG_THC

	// for V3 - Laser and router are using same Output PIN
	// LCD Reset using same pin

#endif



//#define USE_EEPROM

// custom
//#define spindle_pin D1

/*
  ============================================================================================
    CONFIGURATION
  ============================================================================================
*/

//#define PLOTTING
//#define MESHLEVEL
//#define ARC_SUPPORT // 3kb

//#define CHANGEFILAMENT //580byte
#define HARDSTOP // allow to stop in the middle of movement, and still keep the current position, great for CNC
#define WIFISERVER
#define USEOTA
//#define motorz_servo






//#define TOOLON HIGH // depends on laser machine


//#define USE_EEPROM

#ifdef powerpin
	#define POWERFAILURE
#endif

// lets assume if not tool1_pin not defined use the heater_pin

// to make sure all board can be user for laser engraving
//#define tool1_pin heater_pin
//#define LASERMODE


#if defined(__AVR__) || defined (__ARM__)
	#warning Doesnot support ARM/AVR
#endif

//#define motortimeout 10000000 // 10 seconds


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


//#define AUTO_MOTOR_Z_OFF



//#define INVERTENDSTOP // uncomment for normally open

#define ENDSTOP_MOVE 3   //2mm move back after endstop hit, warning, must
#define HOMING_MOVE 2000

#include "platform.h"
#include "motors.h"

#endif
