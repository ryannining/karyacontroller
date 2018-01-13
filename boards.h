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
#define temp_pin 6 //analog 6
#define heater_pin 3


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
//#define ymiaxn_pin 15

//#define zmin_pin 18
#define zmax_pin 19

#define temp_pin 13
#define heater_pin 3


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
 * /*
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

#else
#warning No BOARD Defined !
#endif
