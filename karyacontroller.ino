
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
int kdl = 500;
int tmax,tmin;
void gcode_loop() {

#ifndef ISPC

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
        kdl = 1000;
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
#ifdef timing
  uint32_t t1 = micros();
#endif
#ifndef USETIMER1
  if (motionloop())
#endif
  {
#ifdef timing
    uint32_t t2 = micros()-t1;
    tmax=fmax(t2,tmax);
    tmin=fmin(t2,tmin);
    
    if (ct++ > 1) {
      zprintf(PSTR("%d %dus\n"), fi(tmin),fi(tmax));
      tmax=0;
      tmin=1000;
      ct = 0;
    }
#endif
  }

  if (ack_waiting) {
    zprintf(PSTR("ok\n"));
    ack_waiting = 0;
    n = 1;
  }


  char c = 0;
  if (Serial.available() > 0)
  {
    if (n) {
      gt = micros();
      n = 0;
    }
    c = Serial.read();
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
    Serial.write(c);
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
  Serial.begin(115200);//115200);
  //while (!Serial.available())continue;
#ifdef USE_SDCARD
  demoSD();
#endif
  initmotion();
  init_gcode();
  init_temp();
  reload_eeprom();
  zprintf(PSTR("start\nok\n"));
#ifdef KBOX_PIN
  pinMode(KBOX_PIN, INPUT_PULLUP);
#ifdef ISRTEMP
  vanalog[KBOX_PIN] = 1023; // first read is error, throw it
#endif
#endif

  setPeriod(5000);
  //zprintf(PSTR("Motion demo\nok\n"));

}

void loop() {
  //demo();
  gcode_loop();
  /*  if (feedthedog()){
      float f=123.456;
      int32_t i=1234;
      xprintf("%f",ff(f),i);
    }
  */
}
