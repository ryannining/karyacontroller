#ifndef EEPROM_H
#define EEPROM_H

#include "motion.h"
#include "common.h"



#if defined( __AVR__)
#include <avr/eeprom.h>

#define eepromread(p) eeprom_read_dword((uint32_t)&p)
#define eepromwrite(p,val) eeprom_write_dword((uint32_t)&p,(int32_t)val)
#define eepromcommit
#define eeprominit

#elif defined(ESP8266) ///end avr

#include <EEPROM.h>
#define EEMEM
#define eepromwrite(p,val) EEPROM.put(p, (int32_t)val)
static int32_t eepromread(int p) {
  int32_t val;
  EEPROM.get(p, val);
  return val;
}
static void  eepromwritestring(int p, char* str) {
  byte l = strlen(str);
  EEPROM.put(p, l);
  for (int i = 0; i < l; i++) {
    p++;
    EEPROM.put(p, str[i]);
  }
}
static int  eepromreadstring(int p, char* str,int len) {
  int i;
  byte l;
  
  EEPROM.get(p, l);
  if (l>len)l=len;
  if (l<0)l=0;
  for (i = 0; i < l; i++) {
    p++;
    EEPROM.get(p, str[i]);
  }
  str[l] = 0;
}

#define eepromcommit EEPROM.commit()
#define eeprominit EEPROM.begin(512);

#elif defined(__ARM__) // esp8266
#include <EEPROM.h>
#define EEMEM


static int32_t eepromread(int p)
{
  int32_t r;
  uint16_t* b;
  b = (uint16_t*)&r;
  EEPROM.read(p, b); p += 2; b++;
  EEPROM.read(p, b);
  //zprintf(PSTR("Read eeprom %d %d\n"),fi(p),fi(r));
  return r;
}
static void eepromwrite(int p, int32_t val)
{
  uint16_t* b;
  b = (uint16_t*)&val;
  EEPROM.update(p, *b); p += 2; b++;
  EEPROM.update(p, *b);
  //zprintf(PSTR("Write eeprom %d %d\n"),fi(p),fi(val));
}

#define eepromcommit
#define eeprominit EEPROM.PageBase0 = 0x801F000; EEPROM.PageBase1 = 0x801F800; EEPROM.PageSize  = 0x400;EEPROM.init();
#endif //


#ifndef __AVR__
/*
  #define EE_home 145
  #define EE_home 149
  #define EE_home 153
  #define EE_accelx 51
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

#define EE_xhome 0
#define EE_yhome 5
#define EE_zhome 10
#define EE_accelx 15
#define EE_accely 20
#define EE_accelz 25
#define EE_accele 35
#define EE_mvaccelx 40
#define EE_mvaccely 45
#define EE_mvaccelz 50
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
#ifdef NONLINEAR
#define EE_hor_radius 155
#define EE_rod_length 160
#endif

#define EE_jerk 165
#define EE_xyscale 170

#ifdef POWERFAILURE
#define EE_lastline 200
#endif

#define EE_telebot 380
#define EE_wifi_ap 400
#define EE_wifi_pwd 450
#define EE_wifi_dns 470


#define EE_retract_in 300
#define EE_retract_out 305
#define EE_retract_in_f 310
#define EE_retract_out_f 315

#else
extern float EEMEM EE_xhome;
extern float EEMEM EE_yhome;
extern float EEMEM EE_zhome;
extern int32_t EEMEM EE_homing;
extern int32_t EEMEM EE_jerk;
extern float EEMEM EE_xyscale;

extern int32_t EEMEM EE_accelx;

extern int32_t EEMEM EE_mvaccelx;

extern int32_t EEMEM EE_max_x_feedrate;
extern int32_t EEMEM EE_max_y_feedrate;
extern int32_t EEMEM EE_max_z_feedrate;
extern int32_t EEMEM EE_max_e_feedrate;

extern float EEMEM EE_xstepmm;
extern float EEMEM EE_ystepmm;
extern float EEMEM EE_zstepmm;
extern float EEMEM EE_estepmm;

#ifdef NONLINEAR
extern float EEMEM EE_hor_radius;
extern float EEMEM EE_rod_length;
#endif
extern float EEMEM EE_towera_ofs;
extern float EEMEM EE_towerb_ofs;
extern float EEMEM EE_towerc_ofs;

#ifdef USE_BACKLASH
extern int32_t EEMEM EE_xbacklash;
extern int32_t EEMEM EE_ybacklash;
extern int32_t EEMEM EE_zbacklash;
extern int32_t EEMEM EE_ebacklash;
#endif


#ifdef POWERFAILURE
extern int32_t EEMEM EE_lastline;
#endif

#endif


extern void reload_eeprom();
extern void reset_eeprom();
extern char wifi_ap[50];
extern char wifi_pwd[20];
extern char wifi_dns[30];
extern char wifi_telebot[20];

#endif // EEPROM_H
