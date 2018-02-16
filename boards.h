/*
  ============================================================================================
     TARANTHOLE
  ============================================================================================
*/
#ifdef BOARD_TARANTHOLE

#define xenable 2
#define xdirection 6
#define xstep 4

#define yenable 7
#define ydirection 9
#define ystep 8

#define zenable 10
#define zdirection A5
#define zstep A4

#define e0enable 11
#define e0direction A2
#define e0step A3


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define zmax_pin 13
#define temp_pin 6
#define heater_pin 3


/*
  ============================================================================================
     BOARD_NANONANO
  ============================================================================================
*/
#elif defined(BOARD_NANONANO)

#define xenable 2
#define xdirection 6
#define xstep 4

#define yenable 7
#define ydirection 9
#define ystep 8

#define zenable 10
#define zdirection A5
#define zstep A4

#define e0enable 5
#define e0direction A2
#define e0step A3


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define xmin_pin A1
#define ymin_pin A0
#define zmax_pin 13
#define temp_pin 6 //analog 6 because we use ISR
#define heater_pin 3

#define ISRTEMP

/*
  ============================================================================================
     BOARD_NANONANO_DELTA
  ============================================================================================
*/
#elif defined(BOARD_NANONANO_DELTA)


#define DRIVE_DELTA


#define xenable 2
#define xdirection 6
#define xstep 4

#define yenable 2
#define ydirection 9
#define ystep 8

#define zenable 2
#define zdirection A5
#define zstep A4

#define e0enable 2
#define e0direction A2
#define e0step A3



/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define xmax_pin A0
#define ymax_pin A0
#define zmax_pin A0

#define temp_pin 6 //analog 6 because we use ISR
#define heater_pin 3
#define NUMBUFFER 8
#define SDCARD_CS 10
#define INVERTENDSTOP // uncomment for normally open
#define KBOX_PIN 7

#define ISRTEMP
/*
  ============================================================================================
     BOARD_NANONANO_SDCARD
  ============================================================================================
*/
#elif defined(BOARD_NANONANO_SDCARD)

#define xenable 2
#define xdirection 6
#define xstep 4

#define yenable 2
#define ydirection 9
#define ystep 8

#define zenable 2
#define zdirection A5
#define zstep A4

#define e0enable 2
#define e0direction A2
#define e0step A3


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define xmin_pin A0
#define ymin_pin A0
#define zmax_pin A0

#define temp_pin 6 //analog 6 because we use ISR
#define heater_pin 5
//#define fan_pin 5

#define ISRTEMP

#define SDCARD_CS 10
#define INVERTENDSTOP // uncomment for normally open
#define KBOX_PIN 7
//#define USETIMER1

#define NUMBUFFER 13

/*
  ============================================================================================
     BOARD_NANO_3DPLEX
  ============================================================================================
*/
#elif defined(BOARD_NANO_3DPLEX)

#define xenable 2
#define xdirection 6
#define xstep 4

#define yenable 2
#define ydirection 9
#define ystep 8

#define zenable 2
#define zdirection A5
#define zstep A4

#define e0enable 2
#define e0direction A2
#define e0step A3


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

//#define xmin_pin A0
//#define ymin_pin A0
#define zmax_pin A0

#define temp_pin 6 //analog 6 because we use ISR
#define heater_pin 3
//#define fan_pin 5

#define ISRTEMP

#define SDCARD_CS 10
//#define INVERTENDSTOP // uncomment for normally open
#define KBOX_PIN 7
//#define USETIMER1

#define NUMBUFFER 4


/*
  ============================================================================================
     BOARD_NANONANO_STM32
  ============================================================================================
*/
#elif defined(BOARD_NANONANO_STM32)

#define xenable PB9
#define xdirection PB7
#define xstep PB8

#define yenable PB9
#define ydirection PB5
#define ystep PB6

#define zenable PB9
#define zdirection PB3
#define zstep PB4

#define e0enable PB9
#define e0direction PB14
#define e0step PB15


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define xmin_pin PB12
#define ymin_pin PB12
#define zmax_pin PB12
#define INVERTENDSTOP
#define temp_pin PB1
#define heater_pin PA0
#define fan_pin PA2
//#define SDCARD_CS PA4
//#define KBOX_PIN PA3


/*
  ============================================================================================
     BOARD_GEN7
  ============================================================================================
*/
#elif defined(BOARD_GEN7)

#define xenable 25
#define xdirection 28
#define xstep 29

#define yenable 25
#define ydirection 26
#define ystep 27

#define zenable 25
#define zdirection 22
#define zstep 23

#define e0enable 25
#define e0direction 18
#define e0step 19


#define xmin_pin 0
#define ymin_pin 1
#define zmin_pin 2
#define temp_pin A1 //analog 1
#define heater_pin 4
/*
  ============================================================================================
     BOARD_CHCSHIELDV3
  ============================================================================================
*/
#elif defined(BOARD_CHCSHIELDV3)

#define xenable 8
#define xdirection 5
#define xstep 2

#define yenable 8
#define ydirection 6
#define ystep 3

#define zenable 8
#define zdirection 7
#define zstep 4

#define e0enable 8  // set the jumper for 4th motor https://blog.protoneer.co.nz/arduino-cnc-shield-v3-00-assembly-guide/#4THAXIS
#define e0direction 13
#define e0step 12


#define xmin_pin 0
#define ymin_pin 1
#define zmin_pin 2

#define temp_pin 0 //analog 0, on pin ABORT
#define heater_pin A3 // on pin COOLANT


/*
  ============================================================================================
     BOARD_RAMP1.3 MEGA ramps 1.4 is same ??
  ============================================================================================
*/
#elif defined(BOARD_RAMP13)

#define xenable 38
#define xdirection 55
#define xstep 54

#define yenable 56
#define ydirection 61
#define ystep 60

#define zenable 62
#define zdirection 48
#define zstep 46

#define e0enable 24
#define e0direction 28
#define e0step 26


#define xmin_pin 3
//#define xmax_pin 2

#define ymin_pin 14
//#define ymax_pin 15

#define zmin_pin 18
//#define zmax_pin 19

#define temp_pin 13
#define heater_pin 10

#define fan_pin 9
/*
  ============================================================================================
     BOARD_RAMP1.3 MEGA ramps 1.4 is same ??
  ============================================================================================
*/
#elif defined(BOARD_RAMP13_3DPLEX)

#define xenable 38
#define xdirection 55
#define xstep 54

#define yenable 56
#define ydirection 61
#define ystep 60

#define zenable 62
#define zdirection 48
#define zstep 46

#define e0enable 24
#define e0direction 28
#define e0step 26


//#define xmin_pin 3
//#define xmax_pin 2

//#define ymin_pin 14
//#define ymax_pin 15

//#define zmin_pin 18
#define zmax_pin 19

#define temp_pin A13
#define heater_pin 10

#define fan_pin 9

/*
  ============================================================================================
     BOARD_SEMEDIY128AU
  ============================================================================================
*/
#elif defined(BOARD_SEMEDIY128AU)

#define xenable 43//50
#define xstep 42//49
#define xdirection 44 // 51

#define yenable 40//47
#define ystep 39//46
#define ydirection 38//44

#define zenable 33//40
#define zstep 31//38
#define zdirection 29//36

#define e0enable 28//35
#define e0step 30//37
#define e0direction 24//31


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define zmax_pin 13
//#define temp_pin A6
//#define heater_pin 3

/*
   /
  ============================================================================================
     BOARD_SEMEDIY128AU
  ============================================================================================
*/
#elif defined(BOARD_SEMEDIYNANO)

#define xenable 13
#define xdirection 11
#define xstep 12

#define yenable 4
#define ydirection 2
#define ystep 3

#define zenable A2
#define zdirection A0
#define zstep A1

#define e0enable A5
#define e0direction A3
#define e0step A4


/*#define xmin_pin 0
  #define xmax_pin 0
  #define ymin_pin 0
  #define ymax_pin 0
  #define zmin_pin 0
*/

#define xmin_pin 9
#define ymin_pin 8
#define zmax_pin 7
#define temp_pin 6
#define heater_pin 10
/*
  ============================================================================================
     ESP01CNC_V1
  ============================================================================================
*/
#elif defined(BOARD_ESP01CNC_V1)

#define xenable D1
#define xdirection D2
#define xstep D3
#define yenable D1
#define ydirection D2
#define ystep D3
#define zenable D1
#define zdirection D2
#define zstep D3

#define zmax_pin D5

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

#define zmax_pin D1
#define xmin_pin D1
#define ymin_pin D1

#define temp_pin A0
#define heater_pin D4
#define fan_pin D4

#define INVERTENDSTOP
#define SDCARD_CS D8
#define NUMBUFFER 20
#else
#error No BOARD Defined !
#endif
