
//#define timing
//#define timingG
//#define echoserial


#include "config_pins.h"
#include "common.h"
#include "gcode.h"
#include "temp.h"
#include "timer.h"
#include "eprom.h"
#include "motion.h"
//#include "u8glib_ex.h"
#include<stdint.h>

int line_done, ack_waiting = 0;
int ct = 0;
uint32_t gt = 0;
int n = 0;
uint32_t kctr = 0;
int akey = 0, lkey = 0;
int kdl = 200;



/*
 * 
 * 
 * 
 * 
 */
#ifdef OLEDDISPLAY
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

void menu_up(){

}
void menu_down(){

}
void menu_click(){

}
void menu_back(){

}

#define KBOX_KEY_ACT(k)   case k: KBOX_KEY##k##_ACTION;break;
#define KBOX_KEY1_ACTION menu_up()
#define KBOX_KEY2_ACTION menu_down()
#define KBOX_KEY3_ACTION menu_click()
#define KBOX_KEY4_ACTION menu_back()

#define KBOX_DO_ACT  KBOX_KEY_ACT(1) KBOX_KEY_ACT(2) KBOX_KEY_ACT(3) KBOX_KEY_ACT(4)


SSD1306AsciiAvrI2c oled;
void oledwr(uint8_t c){
  oled.write(c);
}
#define oprintf(...)   sendf_P(oledwr, __VA_ARGS__)
#define gotoxy(x,y) oled.setCursor(x,y)
#define gotox(x) oled.setCol(x)
#define gotoy(y) oled.setRow(y)
#define f1x() oled.set1X()
#define f2x() oled.set2X()
#define oclear() oled.clear()
#define oclearln() oled.clearToEOL()


//------------------------------------------------------------------------------
void setupdisplay() {

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Callibri14);

  oclear();
  gotoxy(0,0);
  
  oled.print("Karyacontroller\n");
}
void display_loop(){
  gotoxy(0,2);
  oprintf(PSTR("Suhu:%f\n   "),ff(Input));    
  gotoxy(0,4);
  oprintf(PSTR("SDCARD Ok"));    
}
void control_loop(){
#ifdef OLED_CONTROL_PIN

  if (millis() - kctr > kdl) {
    kctr = millis();
#ifdef ISRTEMP
    int key = vanalog[OLED_CONTROL_PIN];
    ADCREAD(OLED_CONTROL_PIN)
#else
    int key = analogRead(OLED_CONTROL_PIN);
#endif

    switch (key) {
        KBOX_DO_CHECK  // standart 4 key control box
 
      case 1021 ... 1023:
        if (lkey) {
          zprintf(PSTR("LKey:%d\n"), fi(lkey));
          switch (lkey) {
            KBOX_DO_ACT
          }
        }
        lkey = 0;
        kdl = 200;
        break;
    }
    //zprintf(PSTR("Key:%d\n"), fi(key));
  }
#endif
  
}
#else
#define setupdisplay()
#define display_loop()
#define control_loop()

#endif
/*
      =========================================================================================================================================================
 */
int tmax,tmin;
uint32_t dbtm,dbcm=0;
void gcode_loop() {

#ifndef ISPC

///*
    if (micros() - dbtm> 1000000) {
      dbtm=micros();
      extern int32_t dlp;
      extern int subp;
      float f=(cmctr-dbcm);
      f/=XSTEPPERMM;
      
      //if (f>0)
      zprintf(PSTR("Subp:%d STEP:%d %f %d\n"),fi(subp),cmctr,ff(f),fi(dlp/8));
      dbcm=cmctr;
    }
//*/

  /*
      =========================================================================================================================================================

       KONTROLBOX

      =========================================================================================================================================================
  */
#ifdef KBOX_PIN

  if (millis() - kctr > kdl) {
    kctr = millis();
#ifdef ISRTEMP
    int key = vanalog[KBOX_PIN];
    ADCREAD(KBOX_PIN)
#else
    int key = analogRead(KBOX_PIN);
#endif

    switch (key) {
        KBOX_DO_CHECK

      case 1021 ... 1023:
        if (lkey) {
          zprintf(PSTR("LKey:%d\n"), fi(lkey));
          switch (lkey) {
              KBOX_DO_ACT

          }
        }
        lkey = 0;
        kdl = 200;
        break;
    }
    //zprintf(PSTR("Key:%d\n"), fi(key));
  }
#endif
  /*
      =========================================================================================================================================================
  */

  /*
      =========================================================================================================================================================

       MOTIONLOOP

      =========================================================================================================================================================
  */
    int32_t zz;
#ifdef timing
  uint32_t t1 = micros();
  if (motionloop())
  {
    uint32_t t2 = micros()-t1;
    tmax=fmax(t2,tmax);
    tmin=fmin(t2,tmin);
    
    if (ct++ > 1) {
      zprintf(PSTR("%d %dus\n"), fi(tmin),fi(tmax));
      tmax=0;
      tmin=1000;
      ct = 0;
    }
  }
#else  
  for (int i=0;i<5;i++){if (!motionloop())break;}
#endif
  servo_loop();
  if (ack_waiting) {
    zprintf(PSTR("ok\n"));
    ack_waiting = 0;
    n = 1;
  }


  char c = 0;
  if (serialav())
  {
    if (n) {
      gt = micros();
      n = 0;
    }
    serialrd(c);
  }
#ifdef USE_SDCARD
  if (sdcardok == 2) {
    // read from the file until there's nothing else in it:
    if (myFile.available()) {
      c = myFile.read();
      //myFile.write(c);
    } else {
      // close the file:
      myFile.close();
      sdcardok = 0;
      zprintf(PSTR("Done\n"));
      c = 0;
    }
  }
#endif

  if (c) {
#ifdef echoserial
    serialwr(c);
#endif
    line_done = gcode_parse_char(c);
    if (line_done) {
      ack_waiting = line_done - 1;
#ifdef timingG
      zprintf(PSTR("Gcode:%dus\n"), fi(micros() - gt));
#endif
    }
  }
#else
#endif

}

void setup() {
  // put your setup code here, to run once:
  //  Serial.setDebugOutput(true);
  serialinit(115200);//115200);
  //while (!Serial.available())continue;
#ifdef USE_SDCARD
  demoSD();
#endif
  initmotion();
  init_gcode();
  init_temp();
  reload_eeprom();
  timer_init();
  servo_init();
  SEI
  zprintf(PSTR("start\nok\n"));
#ifdef KBOX_PIN
  pinMode(KBOX_PIN, INPUT_PULLUP);
#ifdef ISRTEMP
  vanalog[KBOX_PIN] = 1023; // first read is error, throw it
#endif
#endif

  //timer_set(5000);
  //zprintf(PSTR("Motion demo\nok\n"));
  setupdisplay();
}

void loop() {
  //demo();
  gcode_loop();
  control_loop();
  display_loop();
  /*  if (feedthedog()){
      float f=123.456;
      int32_t i=1234;
      xprintf("%f",ff(f),i);
    }
  */
}
