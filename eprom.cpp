#include "motion.h"
#include "config_pins.h"
#include "common.h"

#ifdef USE_EEPROM
#include<avr/eeprom.h>

float EEMEM EE_xmax;
float EEMEM EE_ymax;
float EEMEM EE_zmax;

int32_t EEMEM EE_accelx;
int32_t EEMEM EE_accely;
int32_t EEMEM EE_accelz;
int32_t EEMEM EE_accele;

int32_t EEMEM EE_mvaccelx;
int32_t EEMEM EE_mvaccely;
int32_t EEMEM EE_mvaccelz;
int32_t EEMEM EE_mvaccele;

int32_t EEMEM EE_max_x_feedrate;
int32_t EEMEM EE_max_y_feedrate;
int32_t EEMEM EE_max_z_feedrate;
int32_t EEMEM EE_max_e_feedrate;

float EEMEM EE_xstepmm;
float EEMEM EE_ystepmm;
float EEMEM EE_zstepmm;
float EEMEM EE_estepmm;

int32_t EEMEM EE_xjerk;
int32_t EEMEM EE_yjerk;
int32_t EEMEM EE_zjerk;
int32_t EEMEM EE_ejerk;


void reload_eeprom(){
        ax_max[0]=(float)eeprom_read_dword((uint32_t *) &EE_xmax)/1000;
        ax_max[1]=(float)eeprom_read_dword((uint32_t *) &EE_ymax)/1000;
        ax_max[2]=(float)eeprom_read_dword((uint32_t *) &EE_zmax)/1000;
        accel[0]=eeprom_read_dword((uint32_t *) &EE_accelx);
        accel[1]=eeprom_read_dword((uint32_t *) &EE_accely);
        accel[2]=eeprom_read_dword((uint32_t *) &EE_accelz);
        accel[3]=eeprom_read_dword((uint32_t *) &EE_accele);

        mvaccel[0]=eeprom_read_dword((uint32_t *) &EE_mvaccelx);
        mvaccel[1]=eeprom_read_dword((uint32_t *) &EE_mvaccely);
        mvaccel[2]=eeprom_read_dword((uint32_t *) &EE_mvaccelz);
        mvaccel[2]=accel[3];
        
        maxf[0]=eeprom_read_dword((uint32_t *) &EE_max_x_feedrate);
        maxf[1]=eeprom_read_dword((uint32_t *) &EE_max_y_feedrate);
        maxf[2]=eeprom_read_dword((uint32_t *) &EE_max_z_feedrate);
        maxf[3]=eeprom_read_dword((uint32_t *) &EE_max_e_feedrate);
        
         stepmmx[0]=(float)eeprom_read_dword((uint32_t *) &EE_xstepmm)/1000;
         stepmmx[1]=(float)eeprom_read_dword((uint32_t *) &EE_ystepmm)/1000;
         stepmmx[2]=(float)eeprom_read_dword((uint32_t *) &EE_zstepmm)/1000;
         stepmmx[3]=(float)eeprom_read_dword((uint32_t *) &EE_estepmm)/1000;
 
 
         jerk[0]=eeprom_read_dword((uint32_t *) &EE_xjerk);
         jerk[1]=eeprom_read_dword((uint32_t *) &EE_yjerk);
         jerk[2]=eeprom_read_dword((uint32_t *) &EE_zjerk);
         jerk[3]=eeprom_read_dword((uint32_t *) &EE_ejerk);
             
}

void reset_eeprom(){
        reset_motion();
        eeprom_write_dword((uint32_t *) &EE_xmax,ff(ax_max[0]));
        eeprom_write_dword((uint32_t *) &EE_ymax,ff(ax_max[1]));
        eeprom_write_dword((uint32_t *) &EE_zmax,ff(ax_max[2]));
        
        eeprom_write_dword((uint32_t *) &EE_accelx,fi(accel[0]));
        eeprom_write_dword((uint32_t *) &EE_accely,fi(accel[1]));
        eeprom_write_dword((uint32_t *) &EE_accelz,fi(accel[2]));
        eeprom_write_dword((uint32_t *) &EE_accele,fi(accel[3]));

        eeprom_write_dword((uint32_t *) &EE_mvaccelx,fi(mvaccel[0]));
        eeprom_write_dword((uint32_t *) &EE_mvaccely,fi(mvaccel[1]));
        eeprom_write_dword((uint32_t *) &EE_mvaccelz,fi(mvaccel[2]));

        eeprom_write_dword((uint32_t *) &EE_max_x_feedrate,fi(maxf[0]));
        eeprom_write_dword((uint32_t *) &EE_max_y_feedrate,fi(maxf[1]));
        eeprom_write_dword((uint32_t *) &EE_max_z_feedrate,fi(maxf[2]));
        eeprom_write_dword((uint32_t *) &EE_max_e_feedrate,fi(maxf[3]));
        
         eeprom_write_dword((uint32_t *) &EE_xstepmm,ff(stepmmx[0]));
         eeprom_write_dword((uint32_t *) &EE_ystepmm,ff(stepmmx[1]));
         eeprom_write_dword((uint32_t *) &EE_zstepmm,ff(stepmmx[2]));
         eeprom_write_dword((uint32_t *) &EE_estepmm,ff(stepmmx[3]));
 
         eeprom_write_dword((uint32_t *) &EE_xjerk,fi(jerk[0]));
         eeprom_write_dword((uint32_t *) &EE_yjerk,fi(jerk[1]));
         eeprom_write_dword((uint32_t *) &EE_zjerk,fi(jerk[2]));
         eeprom_write_dword((uint32_t *) &EE_ejerk,fi(jerk[3]));
  
}

#endif
