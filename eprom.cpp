#include "motion.h"
#include "config_pins.h"
#include "common.h"



#ifdef USE_EEPROM
#include "eprom.h"

#ifdef __AVR__
float EEMEM EE_xmax;
float EEMEM EE_ymax;
float EEMEM EE_zmax;
int32_t EEMEM EE_homing;
int32_t EEMEM EE_jerk;


int32_t EEMEM EE_accelx;
int32_t EEMEM EE_mvaccelx;


int32_t EEMEM EE_max_x_feedrate;
int32_t EEMEM EE_max_y_feedrate;
int32_t EEMEM EE_max_z_feedrate;
int32_t EEMEM EE_max_e_feedrate;

float EEMEM EE_xstepmm;
float EEMEM EE_ystepmm;
float EEMEM EE_zstepmm;
float EEMEM EE_estepmm;
float EEMEM EE_xyscale;

#ifdef USE_BACKLASH
int32_t EEMEM EE_xbacklash;
int32_t EEMEM EE_ybacklash;
int32_t EEMEM EE_zbacklash;
int32_t EEMEM EE_ebacklash;
#endif

#ifdef NONLINEAR
float EEMEM EE_hor_radius;
float EEMEM EE_rod_length;
float EEMEM EE_towera_ofs;
float EEMEM EE_towerb_ofs;
float EEMEM EE_towerc_ofs;
#endif

#endif

void reload_eeprom() {
  eepromcommit;
  
  ax_max[0] = (float)eepromread(EE_xmax) * 0.001;
  ax_max[1] = (float)eepromread(EE_ymax) * 0.001;
  ax_max[2] = (float)eepromread(EE_zmax) * 0.001;
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

  xyjerk=eepromread(EE_jerk);
  homingspeed=eepromread(EE_homing);
  xyscale = (float)eepromread(EE_xyscale) * 0.001;
#ifdef NONLINEAR
  delta_radius= (float)eepromread(EE_hor_radius)   * 0.001;
  delta_diagonal_rod= (float)eepromread(EE_rod_length)   * 0.001;
  axisofs[0]=(float)eepromread(EE_towera_ofs)   * 0.001;
  axisofs[1]=(float)eepromread(EE_towerb_ofs)   * 0.001;
  axisofs[2]=(float)eepromread(EE_towerc_ofs)   * 0.001;
#endif

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
  eepromwrite(EE_xmax, ff(ax_max[0]));
  eepromwrite(EE_ymax, ff(ax_max[1]));
  eepromwrite(EE_zmax, ff(ax_max[2]));

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

  eepromwrite(EE_homing,homingspeed);
  eepromwrite(EE_jerk,xyjerk);
  eepromwrite(EE_xyscale,ff(xyscale));
#ifdef NONLINEAR
  eepromwrite(EE_hor_radius,ff(delta_radius));
  eepromwrite(EE_rod_length,ff(delta_diagonal_rod));
  eepromwrite(EE_towera_ofs,ff(axisofs[0]));
  eepromwrite(EE_towerb_ofs,ff(axisofs[1]));
  eepromwrite(EE_towerc_ofs,ff(axisofs[2]));
#endif

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
