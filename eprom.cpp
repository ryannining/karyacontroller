#include "motion.h"
#include "config_pins.h"
#include "common.h"



#ifdef USE_EEPROM
#include "eprom.h"

#ifdef __AVR__
float EEMEM EE_xmax;
float EEMEM EE_ymax;
float EEMEM EE_zmax;

int32_t EEMEM EE_accelx;
int32_t EEMEM EE_mvaccelx;

int32_t EEMEM EE_jerkxy;
int32_t EEMEM EE_jerkz;

int32_t EEMEM EE_max_x_feedrate;
int32_t EEMEM EE_max_y_feedrate;
int32_t EEMEM EE_max_z_feedrate;
int32_t EEMEM EE_max_e_feedrate;

float EEMEM EE_xstepmm;
float EEMEM EE_ystepmm;
float EEMEM EE_zstepmm;
float EEMEM EE_estepmm;

#ifdef USE_BACKLASH
int32_t EEMEM EE_xbacklash;
int32_t EEMEM EE_ybacklash;
int32_t EEMEM EE_zbacklash;
int32_t EEMEM EE_ebacklash;
#endif

#ifdef DRIVE_DELTA
int32_t EEMEM DT_DIAGONAL_ROD;
int32_t EEMEM DT_RADIUS;
int32_t EEMEM DT_TOWERA_OFFSET;
int32_t EEMEM DT_TOWERB_OFFSET;
int32_t EEMEM DT_TOWERC_OFFSET;
#endif

#endif

void reload_eeprom() {
  eepromcommit;
  
  ax_max[0] = (float)eepromread(EE_xmax) * 0.01;
  ax_max[1] = (float)eepromread(EE_ymax) * 0.01;
  ax_max[2] = (float)eepromread(EE_zmax) * 0.01;
  accel = eepromread(EE_accelx);

  mvaccel = eepromread(EE_mvaccelx);

  maxf[0] = eepromread(EE_max_x_feedrate);
  maxf[1] = eepromread(EE_max_y_feedrate);
  maxf[2] = eepromread(EE_max_z_feedrate);
  maxf[3] = eepromread(EE_max_e_feedrate);

  stepmmx[0] = (float)eepromread(EE_xstepmm)  * 0.001;
  stepmmx[1] = (float)eepromread(EE_ystepmm)  * 0.001;
  stepmmx[2] = (float)eepromread(EE_zstepmm)   * 0.001;
  stepmmx[3] = (float)eepromread(EE_estepmm)   * 0.001;


#ifdef USE_BACKLASH
  xback[0] = eepromread(EE_xbacklash);
  xback[1] = eepromread(EE_ybacklash);
  xback[2] = eepromread(EE_zbacklash);
  xback[3] = eepromread(EE_ebacklash);

#endif
  preparecalc();
}

void reset_eeprom() {
#ifndef SAVE_RESETMOTION
  reset_motion();
  eepromwrite(EE_xmax, fg(ax_max[0]));
  eepromwrite(EE_ymax, fg(ax_max[1]));
  eepromwrite(EE_zmax, fg(ax_max[2]));

  eepromwrite(EE_accelx, fi(accel));
  
  eepromwrite(EE_mvaccelx, fi(mvaccel));

  eepromwrite(EE_max_x_feedrate, fi(maxf[0]));
  eepromwrite(EE_max_y_feedrate, fi(maxf[1]));
  eepromwrite(EE_max_z_feedrate, fi(maxf[2]));
  eepromwrite(EE_max_e_feedrate, fi(maxf[3]));

  eepromwrite(EE_xstepmm, ff(stepmmx[0]));
  eepromwrite(EE_ystepmm, ff(stepmmx[1]));
  eepromwrite(EE_zstepmm, ff(stepmmx[2]));
  eepromwrite(EE_estepmm, ff(stepmmx[3]));

#ifdef USE_BACKLASH
  eepromwrite(EE_xbacklash, fi(xback[0]));
  eepromwrite(EE_ybacklash, fi(xback[1]));
  eepromwrite(EE_zbacklash, fi(xback[2]));
  eepromwrite(EE_ebacklash, fi(xback[3]));
#endif
  eepromcommit;
#endif  
}
#else
void reload_eeprom() {}
void reset_eeprom() {}

#endif
