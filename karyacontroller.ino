
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
#include<stdint.h>

int line_done, ack_waiting = 0;
int ct = 0;
uint32_t gt = 0;
int n = 0;
uint32_t kctr = 0;
int akey = 0, lkey = 0;
int kdl = 200;



/*




*/
#ifdef LCDDISPLAY
#include  <Wire.h>
#include  <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(LCDDISPLAY, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

void menu_up() {

}
void menu_down() {

}
void menu_click() {

}
void menu_back() {

}

#define KBOX_KEY_ACT(k)   case k: KBOX_KEY##k##_ACTION;break;
#define KBOX_KEY1_ACTION menu_up()
#define KBOX_KEY2_ACTION menu_down()
#define KBOX_KEY3_ACTION menu_click()
#define KBOX_KEY4_ACTION menu_back()

#define KBOX_DO_ACT  KBOX_KEY_ACT(1) KBOX_KEY_ACT(2) KBOX_KEY_ACT(3) KBOX_KEY_ACT(4)


void oledwr(uint8_t c) {
  lcd.write(c);
}
#define oprintf(...)   sendf_P(oledwr, __VA_ARGS__)
#define gotoxy(x,y) lcd.setCursor(x,y)
#define oclear() lcd.clear()


//------------------------------------------------------------------------------
void setupdisplay() {

  lcd.begin();                      // initialize the lcd
  lcd.backlight();
  gotoxy(0, 0);
  oprintf(PSTR("Karyacontroller"));
}
uint32_t sw, next_lcd = 0;
void display_loop() {

  uint32_t lcm = millis();
  if (lcm - next_lcd < 500) return;
  next_lcd = lcm; // each half second
  if (sw++ > 3)sw = 0;
  switch (sw) {
    case 0:
      gotoxy(0, 0);
#ifdef USE_SDCARD
      if (sdcardok == 1) {
        oprintf(PSTR("SD:%d lines"), fi(linecount));
      } else if (sdcardok == 2) {
        oprintf(PSTR("Printing:%d"), fi(lineprocess));
      } else
#endif
      {
        oprintf(PSTR("Suhu:%f         "), ff(Input));
      }
      break;
    case 1:
      gotoxy(0, 1);
      //              ----------------
#ifdef USE_SDCARD
      if (sdcardok == 1) {
        oprintf(PSTR("Home Heat Prn -"));
      }
      else if (sdcardok == 2) {
        oprintf(PSTR("- -  Pause Off"));
      }
#endif
      {
        oprintf(PSTR("Home Heat SD Off"));
      }
      break;
  }
}
void control_loop() {
  /*
    #ifdef OLED_CONTROL_PIN

    if (millis() - kctr > kdl) {
    kctr = millis();
    #ifdef ISRTEMP
    int key = vanalog[OLED_CONTROL_PIN];
    ADCREAD(OLED_CONTROL_PIN)
    #else
    int key = analogRead(OLED_CONTROL_PIN) >> ANALOGSHIFT;
    #endif

    switch (key) {
        KBOX_DO_CHECK  // standart 4 key control box

      case 1021 ... 1023:
        if (lkey) {
          zprintf(PSTR("KKey:%d\n"), fi(lkey));
          switch (lkey) {
            KBOX_DO_ACT
          }
        }
        lkey = 0;
        kdl = 200;
        break;
    }
    zprintf(PSTR("Key:%d\n"), fi(key));
    }
    #endif
  */
}
#else
#define setupdisplay()
#define display_loop()
#define control_loop()

#endif
/*
      =========================================================================================================================================================
*/
int tmax, tmin;
uint32_t dbtm, dbcm = 0;
void gcode_loop() {

#ifndef ISPC

  ///*
  if (micros() - dbtm > 1000000) {
    dbtm = micros();
    extern int32_t dlp;
    extern int subp;
    float f = (cmctr - dbcm);
    f /= XSTEPPERMM;

    //if (f>0)
    //zprintf(PSTR("Subp:%d STEP:%d %f %d\n"),fi(subp),cmctr,ff(f),fi(dlp/8));
    dbcm = cmctr;
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
    int key = analogRead(KBOX_PIN) >> ANALOGSHIFT;
#endif

    switch (key) {
        KBOX_DO_CHECK
      case 1021 ... 1023:
        if (lkey) {
          //zprintf(PSTR("LKey:%d\n"), fi(lkey));
          switch (lkey) {
            case 4: zprintf(PSTR("HOMING\n")); homing(); break;
            case 3: zprintf(PSTR("HEATING\n")); set_temp(190); break;
            case 2: if (sdcardok) {
                sdcardok = sdcardok == 1 ? 2 : 1;
                zprintf(PSTR("SD\n"));
              } else demoSD(); break;
            case 1: RUNNING = 0; sdcardok = 0; zprintf(PSTR("STOP\n")); power_off(); break;

          }
        }
        lkey = 0;
        kdl = 200;
        break;
    }
#ifdef KBOX_SHOW_VALUE
    zprintf(PSTR("Key:%d\n"), fi(key));
#endif

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
    uint32_t t2 = micros() - t1;
    tmax = fmax(t2, tmax);
    tmin = fmin(t2, tmin);

    if (ct++ > 1) {
      zprintf(PSTR("%d %dus\n"), fi(tmin), fi(tmax));
      tmax = 0;
      tmin = 1000;
      ct = 0;
    }
  }
#else
  for (int i = 0; i < 5; i++) {
    if (!motionloop())break;
  }
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
    if (c == '\n')lineprocess++;
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
#ifdef __ARM__
  pinMode(KBOX_PIN, INPUT_ANALOG);
#else
  pinMode(KBOX_PIN, INPUT_PULLUP);
#endif
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
