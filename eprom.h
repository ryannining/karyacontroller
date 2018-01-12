#ifdef __AVR__
#else
#define EEMEM
#endif
extern void reload_eeprom();
extern void reset_eeprom();

extern float EEMEM EE_xmax;
extern float EEMEM EE_ymax;
extern float EEMEM EE_zmax;

extern int32_t EEMEM EE_accelx;
extern int32_t EEMEM EE_mvaccelx;
extern int32_t EEMEM EE_accely;
extern int32_t EEMEM EE_mvaccely;
extern int32_t EEMEM EE_accelz;
extern int32_t EEMEM EE_mvaccelz;
extern int32_t EEMEM EE_accele;
extern int32_t EEMEM EE_mvaccele;

extern int32_t EEMEM EE_max_x_feedrate;
extern int32_t EEMEM EE_max_y_feedrate;
extern int32_t EEMEM EE_max_z_feedrate;
extern int32_t EEMEM EE_max_e_feedrate;

extern float EEMEM EE_xstepmm;
extern float EEMEM EE_ystepmm;
extern float EEMEM EE_zstepmm;
extern float EEMEM EE_estepmm;

extern int32_t EEMEM EE_xjerk;
extern int32_t EEMEM EE_yjerk;
extern int32_t EEMEM EE_zjerk;
extern int32_t EEMEM EE_ejerk;
extern int32_t EEMEM EE_xbacklash;
extern int32_t EEMEM EE_ybacklash;
extern int32_t EEMEM EE_zbacklash;
extern int32_t EEMEM EE_ebacklash;

