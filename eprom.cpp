
#include "common.h"
#include "temp.h"


int wifi_gcode = 0;
String wifi_ap;
String wifi_pwd;
String wifi_dns;


#ifdef USE_EEPROM
//#include "eprom.h"




void reload_eeprom() {
  eepromcommit();
  /*

  ax_home[0] = ((float)eepromread(EE_xhome)) * 0.001;
  ax_home[1] = ((float)eepromread(EE_yhome)) * 0.001;
  ax_home[2] = ((float)eepromread(EE_zhome)) * 0.001;
  accel = eepromread(EE_accel);


  maxf[0] = eepromread(EE_max_x_feedrate);
  maxf[1] = eepromread(EE_max_y_feedrate);
  maxf[2] = eepromread(EE_max_z_feedrate);
  maxf[3] = eepromread(EE_max_e_feedrate);
	*/
  /*
  maxa[0] = accel;
  maxa[1] = accel * 0.5;//maxf[1] / maxf[0] * 0.3;
  maxa[2] = accel * 0.8;//maxf[2] / maxf[0] * 0.3;
  maxa[3] = accel;
  zaccel = maxa[2];
  stepmmx[3] = (float)eepromread(EE_estepmm)   * 0.001;
  perstepx = 1.0 / (stepmmx[0] = (float)eepromread(EE_xstepmm)  * 0.001);
  perstepy = 1.0 / (stepmmx[1] = (float)eepromread(EE_ystepmm)  * 0.001);
  perstepz = 1.0 / (stepmmx[2] = (float)eepromread(EE_zstepmm)  * 0.001);
extern int odir[4];
  odir[0] = stepmmx[0] < 0 ? -1 : 1;
  odir[1] = stepmmx[1] < 0 ? -1 : 1;
  odir[2] = stepmmx[2] < 0 ? -1 : 1;
  odir[3] = stepmmx[3] < 0 ? -1 : 1;

#ifdef ANALOG_THC
  thc_up=eepromread(EE_thc_up);
  thc_ofs=eepromread(EE_thc_ofs);
#endif
*/

/*
  xycorner = eepromread(EE_corner);  
  homingspeed = eepromread(EE_homing);

  Lscale = (float)eepromread(EE_Lscale) * 0.001;

  axisofs[0] = (float)eepromread(EE_towera_ofs)   * 0.001;
  axisofs[1] = (float)eepromread(EE_towerb_ofs)   * 0.001;
  axisofs[2] = (float)eepromread(EE_towerc_ofs)   * 0.001;
*/

/*
#ifdef USE_BACKLASH
  xback[0] = eepromread(EE_xbacklash);
  xback[1] = eepromread(EE_ybacklash);
  xback[2] = eepromread(EE_zbacklash);
  xback[3] = eepromread(EE_ebacklash);

#endif

  retract_in = (float)eepromread(EE_retract_in)   * 0.001;
  retract_out = (float)eepromread(EE_retract_out)   * 0.001;
  retract_in_f = (float)eepromread(EE_retract_in_f)   * 0.001;
  retract_out_f = (float)eepromread(EE_retract_out_f)   * 0.001;

#ifdef WIFISERVER
  wifi_gcode = eepromread(EE_gcode);
  eepromreadstring(400, wifi_ap, 50);
  eepromreadstring(450, wifi_pwd, 20);
  eepromreadstring(470, wifi_dns, 30);
  //eepromreadstring(500, wifi_gcode, 30);
#endif
*/

}

void reset_eeprom() {

  reset_motion();
    /*
  retract_in = 1;
  retract_out = 1;
  retract_in_f = 6;
  retract_out_f = 4;
  

  eepromwrite(EE_xhome, ff(ax_home[0]));
  eepromwrite(EE_yhome, ff(ax_home[1]));
  eepromwrite(EE_zhome, ff(ax_home[2]));

  eepromwrite(EE_accel, fi(accel));



  eepromwrite(EE_max_x_feedrate, fi(maxf[0]));
  eepromwrite(EE_max_y_feedrate, fi(maxf[1]));
  eepromwrite(EE_max_z_feedrate, fi(maxf[2]));
  eepromwrite(EE_max_e_feedrate, fi(maxf[3]));

  eepromwrite(EE_xstepmm, ff(stepmmx[0]));
  eepromwrite(EE_ystepmm, ff(stepmmx[1]));
  eepromwrite(EE_zstepmm, ff(stepmmx[2]));
  eepromwrite(EE_estepmm, ff(stepmmx[3]));
	
  eepromwrite(EE_towera_ofs, ff(axisofs[0]));
  eepromwrite(EE_towerb_ofs, ff(axisofs[1]));
  eepromwrite(EE_towerc_ofs, ff(axisofs[2]));

  eepromwrite(EE_towera_ofs, 0);
  eepromwrite(EE_towerb_ofs, 0);
  eepromwrite(EE_towerc_ofs, 0);

  eepromwrite(EE_retract_in, 0);
  eepromwrite(EE_retract_out, 0);
  eepromwrite(EE_retract_in_f, 10000);
  eepromwrite(EE_retract_out_f, 10000);
*/

/*
#ifdef USE_BACKLASH
  eepromwrite(EE_xbacklash, fi(xback[0]));
  eepromwrite(EE_ybacklash, fi(xback[1]));
  eepromwrite(EE_zbacklash, fi(xback[2]));
  eepromwrite(EE_ebacklash, fi(xback[3]));
#endif

#if defined(heater_pin)
  eepromwrite(EE_pid_p, ff(8.0));
  eepromwrite(EE_pid_i, ff(600.0));
  eepromwrite(EE_pid_d, ff(400.0));
  eepromwrite(EE_pid_bang, ff(4.1));
  eepromwrite(EE_pid_HS, ff(1.0));
#endif
  eepromwrite(EE_ext_adv, ff(0));
  eepromwrite(EE_un_microstep, fi(0));
#endif
*/

  eepromcommit();
}
#else
void reload_eeprom() {}
void reset_eeprom() {}

#endif
