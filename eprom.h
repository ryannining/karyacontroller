#pragma once
#ifndef eprom_H
#define eprom_H

#ifdef USE_EEPROM

#include "timer.h"


#include <EEPROM.h>
#define EEPROM_SIZE 512
#define EEMEM

#define eepromwrite(p,val) EEPROM.put(p, (int32_t)val)

static int32_t eepromread(int p) {
  int32_t val;
  EEPROM.get(p, val);
  //EEPROM.read(p+1, val);
  return val;
}
static void  eepromwritestring(int p, char* str) {
  byte l = strlen(str);
  //EEPROM.write(p, l);
  EEPROM.put(p, l);
  for (int i = 0; i < l; i++) {
    p++;
    //EEPROM.write(p, str[i]);
    EEPROM.put(p, str[i]);
  }
}
static void  eepromreadstring(int p, char* str, int len) {
  int i;
  byte l = 0;

  EEPROM.get(p, l);
  if (l > len)l = len;
  if (l < 0)l = 0;
  for (i = 0; i < l; i++) {
    p++;
    EEPROM.get(p, str[i]);
  }
  str[l] = 0;
}
static void eepromcommit() {
  timerPause();
  //zprintf(PSTR("Commit\n"));
  EEPROM.commit();
  timerResume();
}

#define eeprominit EEPROM.begin(512)


extern int thc_ofs,thc_up,thc_enable;


/*
  #define EE_home 145
  #define EE_home 149
  #define EE_home 153
  #define EE_accel 51
  #define EE_accely 55
  #define EE_accelz 59
  #define EE_accele 63
  #define EE_mvaccelx 67
  #define EE_mvaccely 71
  #define EE_mvaccelz 75
  #define EE_max_x_feedrate 15
  #define EE_max_y_feedrate 19
  #define EE_max_z_feedrate 23
  #define EE_max_e_feedrate 27
  #define EE_xstepmm 3
  #define EE_ystepmm 7
  #define EE_zstepmm 11
  #define EE_estepmm 0

  #define EE_xbacklash 80
  #define EE_ybacklash 84
  #define EE_zbacklash 88
  #define EE_ebacklash 92
*/
/*
#define EE_xhome 0
#define EE_yhome 5
#define EE_zhome 10
#define EE_accel 15
#define EE_corner 20
//#define EE_accely 20
//#define EE_accelz 25
//#define EE_accele 35
#define EE_jerk 40
//#define EE_mvaccely 45
//#define EE_mvaccelz 50
#define EE_max_x_feedrate 55
#define EE_max_y_feedrate 60
#define EE_max_z_feedrate 65
#define EE_max_e_feedrate 70
#define EE_xstepmm 75
#define EE_ystepmm 80
#define EE_zstepmm 85
#define EE_estepmm 90

#define EE_xbacklash 115
#define EE_ybacklash 120
#define EE_zbacklash 125
#define EE_ebacklash 130

#define EE_homing 135
#define EE_towera_ofs 140
#define EE_towerb_ofs 145
#define EE_towerc_ofs 150

#ifdef ANALOG_THC
#define EE_thc_up 155
#define EE_thc_ofs 160
#endif

#define EE_corner 165
#define EE_Lscale 170
*/


#ifdef POWERFAILURE
#define EE_lastline 200
#endif

#define EE_gcode 380
#define EE_wifi_ap 400
#define EE_wifi_pwd 450
#define EE_wifi_dns 470


#define EE_retract_in 300
#define EE_retract_out 305
#define EE_retract_in_f 310
#define EE_retract_out_f 315

#define EE_pid_p 320
#define EE_pid_i 325
#define EE_pid_d 340
#define EE_pid_bang 335
#define EE_pid_HS 355

#define EE_ext_adv 345
#define EE_un_microstep 350
#define EE_softreset 490


extern void reload_eeprom();
extern void reset_eeprom();
extern char wifi_ap[50];
extern char wifi_pwd[20];
extern char wifi_dns[30];
extern int wifi_gcode;

#endif

#endif
