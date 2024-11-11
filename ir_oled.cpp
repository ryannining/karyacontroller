#include "common.h"

bool oledready = false;

static int wait_job = 0;
static int wait_spindle = 0;
static String jobnum;
#include "ir_remote.h"
#include "ir_oled.h"
#ifdef ESP32
#include <SPIFFS.h>
#else
#include <FS.h>
//#define OLEDDISPLAY_REDUCE_MEMORY
#endif

#define LCD_SDA 0
#define LCD_SCL 0

int lcd_rst=0;
int lcd_sda=0;
int lcd_sda2=0;
int lcd_contrast=6;
int lcd_scl=0;
int lcd_addr=0x3c;
int lcd_kind=KIND_NK1661;

float LCD_Y=21;
float LCD_X=7.75;
int YOFS=1;
int XOFS=1;
int LINES=6;
uint32_t mili;


#include "fonts.h"
#include "shaper.h"
extern InputShaper shaper;


KaryaLCD xdisplay(KIND_NK1661); //


#if INCLUDE_I2CLCD
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C LCD(0x27,20,4); // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

#include "gcodesave.h"
#include "gcode.h"
#include "eprom.h"
#include "temp.h"
#include <math.h>

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.

*/

// End of constructor list

//

int menu_index = 1;
int menu_page_le = 0;
String* menu_t;
int* menu_pos;
int* menu_page;
int menu_col;
typedef void (*Tdrawmenu)(void);
typedef void (*Tclickmenu)(int);
Tdrawmenu menu_f;
Tclickmenu menu_click;
Tdrawmenu menu_exit=0;

extern void saveconfigs();
void default_exit(void){
}
void load_menu(int index, String* menu, int le, int col, int* pos, int* page, void f(void), void fc(int))
{
  menu_index = index;
  menu_pos = pos;
  menu_page_le = (le - col) / col - 1;
  menu_page = page;
  menu_t = menu;
  menu_f = f;
  menu_click = fc;
  menu_col = col;
  //REINIT;
  if (menu_exit)menu_exit();
  menu_exit = default_exit;
  if (oledready)f();
}
void draw_menu(void);
void menu_0_click(int a);
void menu_2_click(int a);
void menu_3_click(int a);

void menu_4_click(int a);

#define LEN(x) (sizeof(x) / sizeof(x[0]))
#define LOADINFO() load_menu(99, NULL, 0, 1, &menu_99_pos, &menu_99_page, draw_info, menu_99_click)
#define LOADMENU(xx) load_menu(xx, menu_##xx##_t, LEN(menu_##xx##_t), 1, \
                               &menu_##xx##_pos, &menu_##xx##_page, draw_menu, menu_##xx##_click)

int col2pos = 14;
float vmul = 1;
String jobname;

/*
    MENU 99, INFO MENU

*/

// Standar menu is index 0: title, rest is the menu
int menu_99_pos = 0;
int menu_99_page = 0;
extern int avgpw, avgcnt;

String uptime(long w)
{
  long days = 0;
  long hours = 0;
  long mins = 0;
  long secs = 0;
  secs = w / 1000; //convect milliseconds to seconds
  mins = secs / 60; //convert seconds to minutes
  hours = mins / 60; //convert minutes to hours
  days = hours / 24; //convert hours to days
  secs = secs - (mins * 60); //subtract the coverted seconds to minutes in order to display 59 secs max
  mins = mins - (hours * 60); //subtract the coverted minutes to hours in order to display 59 minutes max
  hours = hours - (days * 24); //subtract the coverted hours to days in order to display 23 hours max
  //Display results
  String ups = "";
  if (days > 0) // days will displayed only if value is greater than zero
  {
    ups = days;
    ups += " day ";
  }
  ups += (hours);
  ups += (":");
  ups += (mins);
  ups += (":");
  ups += (secs);
  return ups;
}
extern int tmul;
extern int rmkey;
extern long lastjobt;
extern int spindle_pct, lastS;
extern long spindle_pwm;
extern void BuzzError(bool v);
uint8_t LCDturn,LCDturnx;
int oldindex=0;
bool showremotekey=true;
void dopause(int tm=0,bool stopping=false);
void draw_info(void)
{
  extern bool inwaitbuffer;
  //extern void update_info();
  //update_info();
  //if (inwaitbuffer)return;

  //if (oldindex!=menu_index){
	  d_clear(); // clear the internal memory
  //  oldindex=menu_index;
  //}	
  d_setcolor(WHITE);
  extern int ISWIFIOK;
  extern String IPA;


  d_text(0, 0, IPA); // write something to the internal memory


//  extern int thcread;
//  d_text(15, 0, String(thcread)); // write something to the internal memory
  if (!RUNNING)
    d_text(15, 0, String(rmkey)); // write something to the internal memory
  else
    d_text(13, 0, "S:"+String(info_ve)); // write something to the internal memory
  
  extern bool isRotary;
  if (isRotary)d_text(10, 0, "Ro"); // write something to the internal memory
  if (shaper.enabled)d_text(8, 0, "iS"); // write something to the internal memory

  //d_frect(0, LCD_Y - 1, d_w() * LCD_X, LCD_Y);
  d_setcolor(WHITE);
  d_text(1, 1, String(info_x)); // write something to the internal memory
  d_text(8, 1, String(info_y-info_x*skew_y*0.001)); // write something to the internal memory
  d_text(15, 1, String(info_z)); // write something to the internal memory
  extern int cmdlaserval,constantlaserVal;
  extern bool toolWasOn;
  
	int r=((cmdlaserval*100)>>8);
  if (cmhead==cmtail)r=((constantlaserVal*100)>>8);
	  if (lasermode){ 
      if (lasermode==1){
        String temp="";
        extern int water,machinefail;
        if (ltemp_pin>-1){
          extern double Input;
          temp="t "+String(int(Input*10)*0.1)+" ";
        }
        d_text(0, 2, "L " + String(r)+"%");
        //temp+=(water>1000?"w !":"w ok"); //String(water);//
        temp+="w "+String(water/100);
        d_text(9,2, temp); // write something to the internal memory
      }
      if (lasermode==2){
	  	  d_text(0, 2, toolWasOn?"PLSM ON":"PLSM OFF"); // write something to the internal memory
      }
    } else {
      d_text(0, 2, "S " + String(r)+"%"); // write something to the internal memory
    }

  d_text(14, (lcd_kind==KIND_LCD2004?2:3), "Fr"+String(int(f_multiplier * 100))); // write something to the internal memory
  String L1, L2, L3;
  if (uncompress) {
    if (PAUSE)L1 = "Paused !"; else
      L1 = jobname;

    L1 += " " + String(100 * gcodepos / gcodesize) + "%";
  }
  else {
    L1 = "+ " + String(tmul) +"%";
  }
  L2 = "On " + uptime(millis());
  L3 = "T " + (lastjobt > 0 ? uptime(lastjobt) : "-");

  LCDturnx++;
  LCDturn=LCDturnx
  #ifdef ESP32
  /3
  #endif
  ;

  if (LINES == 4) {
    //d_text(0, 3, L1);
    if (LCDturn > ((3 << 4) - 1))
      LCDturn = 0;
    if (LCDturn >> 4 == 0)
      d_text(0, 3, L1); // write something to the internal memory
    if (LCDturn >> 4 == 1)
      d_text(0, 3, L2); // write something to the internal memory
    if (LCDturn >> 4 == 2)
      d_text(0, 3, L3); // write something to the internal memory
  }
  else if (LINES == 5) {
    //d_text(0, 3, L1);
    d_text(0, 3, L1);
    if (LCDturn > ((2 << 4) - 1))
      LCDturn = 0;
    if (LCDturn >> 4 == 0)
      d_text(0, 4, L2); // write something to the internal memory
    if (LCDturn >> 4 == 1)
      d_text(0, 4, L3); // write something to the internal memory
  }
  else {
    d_text(0, 3, L1); // write something to the internal memory
    d_text(0, 4, L2);
    d_text(0, 5, L3);
  }

  d_show();
  //tmul=tmr;
}

/*
    MENU 0, HOME MENU

*/
// Standar menu is index 0: title, rest is the menu
int menu_0_pos = 0;
int menu_0_page = 0;
String menu_0_t[] = { "Main Menu", "1. Set Zero", "2. My jobs", "3. Tools", "4. Settings", "5. Retry WIFI" };

/*
    MENU 3, Tools

*/
// Standar menu is index 0: title, rest is the menu
int menu_3_pos = 0;
int menu_3_page = 0;
String menu_3_t[] = { "Tools",  "1. To zero",  "2. To job zero" };

/*
   ONCLICK
*/

Tclickmenu confirm_click;

/*
    MENU 80, Confirmation

*/

// Standar menu is index 0: title, rest is the menu
int menu_80_pos = 0;
int menu_80_page = 0;
String menu_80_t[] = { "", "YES", "NO" };

void menu_80_click(int a)
{
  if (a == IRK_OK || a == IRK_LF || a == IRK4_OK || a == IRK4_LF || a == IRK4_BACK2) {
    int res = (a == IRK_OK || a == IRK4_OK) && (*menu_pos == 0);
    confirm_click(res);
  }
}

#define LOADCONFIRM(T, clickok) \
  menu_80_t[0] = T;           \
  confirm_click = clickok;    \
  LOADMENU(80);

/*

   ============================================================================================
    MENU 4, Settings

*/

/*
    MENU 41, Speed

*/

int menu_4_pos = 0;
int menu_4_page = 0;

// Update menu_4_t array to include new Input Shaper option
String menu_4_t[] = { "Settings", "1. Speed", "2. Acceleration", "3. Backlash", 
                      "4. Step/mm", "5. Homing", "6. Calib. Step", "7. Calib. Skew",
                      "8. THC", "9. Input Shaper", "Save config" };


// Standar menu is index 0: title, rest is the menu
int menu_41_pos = 0;
int menu_41_page = 0;
String menu_41_t[] = { "Max Speed", "mm/s", "Axis X", "50", "Axis Y", "50", "Axis Z", "5", 
        "Tool %", "1" };
int xyzspeed[4];

int getSpeed(void)
{
  xyzspeed[0] = maxf[MX];
  xyzspeed[1] = maxf[MY];
  xyzspeed[2] = maxf[MZ];
  xyzspeed[3] = 100/Lscale;

  menu_41_t[3] = xyzspeed[0];
  menu_41_t[5] = xyzspeed[1];
  menu_41_t[7] = xyzspeed[2];
  menu_41_t[9] = xyzspeed[3];
  return 10;
}

int vamul=1;
int menu_setvalue(int a)
{
  if (a == IRK_LF || a == IRK4_LF)    return -1*vamul;
  if (a == IRK_RG || a == IRK4_RG)    return 1*vamul;
  if (a == IRK_4 || a == IRK4_4)    return -5*vamul;
  if (a == IRK_6 || a == IRK4_6)    return 5*vamul;
  if (a == IRK_X || a == IRK4_X)    return -1000;
  if (a == IRK4_DOT1)    vamul=0.5;
  if (a == IRK4_DOT2)    vamul=1;
  if (a == IRK4_DOT3)    vamul=10;
  if (a == IRK4_DOT4)    vamul=100;  
  return 0;
}

void menu_41_click(int a)
{
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2)
    LOADMENU(4);
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    maxf[MX] = xyzspeed[0];
    maxf[MY] = xyzspeed[1];
    maxf[MZ] = xyzspeed[2];
    Lscale = (float)xyzspeed[3] / 100.0;
    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0) {
    if (v <= -1000) {
      if (*menu_pos == 3)
        xyzspeed[*menu_pos] = -xyzspeed[*menu_pos];
    }
    else {
      xyzspeed[*menu_pos] += v;
      if ((*menu_pos != 3) && (xyzspeed[*menu_pos] < 1))
        xyzspeed[*menu_pos] = 1;
    }
    menu_41_t[(*menu_pos) * 2 + 3] = xyzspeed[*menu_pos];
    draw_menu();
  }
}
void LOADMENU41(void)
{
  col2pos = 13;
  load_menu(41, menu_41_t,
            getSpeed(),
            2,
            &menu_41_pos, &menu_41_page, draw_menu, menu_41_click);
}

int menu_42_pos = 0;
int menu_42_page = 0;
String menu_42_t[] = { "Accel", "mm/s~", "Jerk", "0", "Accel", "250", "Corner", "15" };
int xyzaccel[3];

int getAccel(void)
{
  xyzaccel[0] = 0;
  xyzaccel[1] = accel;
  xyzaccel[2] = xycorner;
  menu_42_t[3] = xyzaccel[0];
  menu_42_t[5] = xyzaccel[1];
  menu_42_t[7] = xyzaccel[2];
  return 8;
}

void menu_42_click(int a)
{
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2)
    LOADMENU(4);
  if (a == IRK_OK || a == IRK4_OK) {


    accel = xyzaccel[1];
    float sc=float(accel)/float(maxa[0]);
    maxa[0]=fmax(50,maxa[0]*sc);
    maxa[1]=fmax(50,maxa[1]*sc);
    maxa[2]=fmax(50,maxa[2]*sc);
    
    xycorner = xyzaccel[2];
    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0 && v > -1000) {
    if (*menu_pos == 0)
      v *= 1000;
    if (*menu_pos == 1)
      v *= 10;

    xyzaccel[*menu_pos] += v;
    int minv = (*menu_pos == 0) ? 0 : 5;
    if (xyzaccel[*menu_pos] < minv)
      xyzaccel[*menu_pos] = minv;
    menu_42_t[(*menu_pos) * 2 + 3] = xyzaccel[*menu_pos];
    draw_menu();
  }
}
void LOADMENU42(void)
{
  col2pos = 12;
  load_menu(42, menu_42_t,
            getAccel(),
            2,
            &menu_42_pos, &menu_42_page, draw_menu, menu_42_click);
}

int menu_43_pos = 0;
int menu_43_page = 0;
String menu_43_t[] = { "Backlash", "mm", "X", "0", "Y", "250", "Z", "15" };
int xyzback[3] = { 10, 10, 3 };

int getBacklash(void)
{
  xyzback[0] = xback[MX]*1000;
  xyzback[1] = xback[MY]*1000;
  xyzback[2] = xback[MZ]*1000;

  menu_43_t[3] = 0.001 * xyzback[0];
  menu_43_t[5] = 0.001 * xyzback[1];
  menu_43_t[7] = 0.001 * xyzback[2];
  col2pos = 14;
  return 8;
}

void menu_43_click(int a)
{
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2)
    LOADMENU(4);
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    //EE_xbacklash
    xback[MX] = xyzback[0]*0.001;
    xback[MY] = xyzback[1]*0.001;
    xback[MZ] = xyzback[2]*0.001;


    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0 && v > -1000) {

    xyzback[*menu_pos] += v * 10;
    if (xyzback[*menu_pos] < 0)
      xyzback[*menu_pos] = 0;
    menu_43_t[(*menu_pos) * 2 + 3] = 0.001 * xyzback[*menu_pos];
    draw_menu();
  }
}
void LOADMENU43(void)
{
  col2pos = 12;
  load_menu(43, menu_43_t,
            getBacklash(),
            2,
            &menu_43_pos, &menu_43_page, draw_menu, menu_43_click);
}

int menu_44_pos = 0;
int menu_44_page = 0;
String menu_44_t[] = { "1 mm =", "Step", "X", "0", "Y", "250", "Z", "15", "SkewY/M","0" };
int xyzsteps[4];

bool cal_skew;
int getSteps(void)
{

  xyzsteps[0] = stepmmx[0] * 1000;
  xyzsteps[1] = stepmmx[1] * 1000;
  xyzsteps[2] = stepmmx[2] * 1000;
  xyzsteps[3] = skew_y*1000;

  menu_44_t[3] = 0.001 * xyzsteps[0];
  menu_44_t[5] = 0.001 * xyzsteps[1];
  menu_44_t[7] = 0.001 * xyzsteps[2];
  menu_44_t[9] = 0.001 * xyzsteps[3];
  return 10;
}

void menu_44_click(int a)
{
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2)
    LOADMENU(4);
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    //EE_estepmm
    stepmmx[0] = xyzsteps[0] * 0.001;
    stepmmx[1] = xyzsteps[1] * 0.001;
    stepmmx[2] = xyzsteps[2] * 0.001;
    
    skew_y = xyzsteps[3] * 0.001;

    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0) {
    if (v == -1000)
      xyzsteps[*menu_pos] = -xyzsteps[*menu_pos];
    else
      xyzsteps[*menu_pos] += v * 10;
      
    menu_44_t[(*menu_pos) * 2 + 3] = 0.001 * xyzsteps[*menu_pos];
    draw_menu();
  }
}

void LOADMENU44(void)
{
  col2pos = 10;
  load_menu(44, menu_44_t,
            getSteps(),
            2,
            &menu_44_pos, &menu_44_page, draw_menu, menu_44_click);
}

int menu_45_pos = 0;
int menu_45_page = 0;
String menu_45_t[] = { "Home Pos", "mm", "X", "0", "Y", "250", "Z", "15" };

int xyzhome[3];
int getHomes(void)
{
  xyzhome[0] = ax_home[0];
  xyzhome[1] = ax_home[1];
  xyzhome[2] = ax_home[2];

  menu_45_t[3] = xyzhome[0];
  menu_45_t[5] = xyzhome[1];
  menu_45_t[7] = xyzhome[2];
  return 8;
}

void menu_45_click(int a)
{
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2) {
    LOADMENU(4);
    //reload_eeprom();
  }
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    //EE_xhome
    ax_home[0] = xyzhome[0];
    ax_home[1] = xyzhome[1];
    ax_home[2] = xyzhome[2];
    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0 && v > -1000) {

    xyzhome[*menu_pos] += v;
    if (xyzhome[*menu_pos] < 0)
      xyzhome[*menu_pos] = 0;
    menu_45_t[(*menu_pos) * 2 + 3] = xyzhome[*menu_pos];
    draw_menu();
  }
}

void LOADMENU45(void)
{
  col2pos = 14;
  load_menu(45, menu_45_t,
            getHomes(),
            2,
            &menu_45_pos, &menu_45_page, draw_menu, menu_45_click);
}
/*
void UPDATEESP(int a)
{
  if (a) {
    extern void updateFirmware();
    updateFirmware();
  }
  LOADMENU(0);
}
*/
int menu_46_pos=0;
int menu_46_page=0;
String menu_46_t[] = { "Actual Pos", "mm", "X", "0", "Y", "0","Z","0"};
int calVal[4];
float skew_yn;
int getCal(void){
  calVal[0]=cx1*1000;
  calVal[1]=cy1*1000;
  calVal[2]=ocz1*1000;
  menu_46_t[3] = cx1;
  menu_46_t[5] = cy1;
  menu_46_t[7] = ocz1;
  return 8;
}

void menu_46_click(int a){
if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2) {
    LOADMENU(4);
   // reload_eeprom();
  }
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    extern float stepmmx[4];
    if (int(cx1*100)!=0)stepmmx[0] *= cx1*1000/calVal[0];
    if (int(cy1*100)!=0)stepmmx[1] *= cy1*1000/calVal[1];
    if (int(ocz1*100)!=0)stepmmx[2] *= ocz1*1000/calVal[2];
    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0) {
    calVal[*menu_pos] += v;     
    menu_46_t[(*menu_pos) * 2 + 3] = 0.001*calVal[*menu_pos];
    draw_menu();
  }
}
void LOADMENU46(void)
{
  col2pos = 14;
  load_menu(46, menu_46_t,
            getCal(),
            2,
            &menu_46_pos, &menu_46_page, draw_menu, menu_46_click);
}
int menu_47_pos=0;
int menu_47_page=0;
String menu_47_t[] = { "Value", "mm","Skew Y","0"};
int getCalSkew(void){
  cal_skew=true;
  skew_yn=skew_y*cx1*0.001;   
  menu_47_t[3] = cy1;
  return 4;
}

void menu_47_click(int a){
if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2) {
    LOADMENU(4);
   // reload_eeprom();
  }
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    extern float stepmmx[4];
    skew_y = (calVal[3]*0.001+skew_yn)*1000.0/float(cx1);
    LOADMENU(4);
  }
}
void LOADMENU47(void)
{
  col2pos = 14;
  load_menu(47, menu_47_t,
            getCalSkew(),
            2,
            &menu_47_pos, &menu_47_page, draw_menu, menu_47_click);
}

#ifdef ANALOG_THC
int menu_48_pos=0;
int menu_48_page=0;
String menu_48_t[] = { "THC", "v", "V.Ref", "0", "Ofset", "0"};
int thcVal[2];
int getThc(void){
  menu_48_t[3] = thc_up;
  menu_48_t[5] = thc_ofs;
  thcVal[0]=thc_up;
  thcVal[1]=thc_ofs;
  return 6;
}

void menu_48_click(int a){
if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2) {
    LOADMENU(4);
   // reload_eeprom();
  }
  if (a == IRK_OK || a == IRK4_OK) {
    // save config and back to menu 4
    //EE_xhome
    thc_up = thcVal[0];
    thc_ofs = thcVal[1];
    LOADMENU(4);
  }
  int v = menu_setvalue(a);
  if (v != 0) {
	if (v == -1000)
      thcVal[*menu_pos] = -thcVal[*menu_pos];
    else
      thcVal[*menu_pos] += v;
      
    menu_48_t[(*menu_pos) * 2 + 3] = thcVal[*menu_pos];
    draw_menu();
  }
}
void LOADMENU48(void)
{
  col2pos = 14;
  load_menu(47, menu_48_t,
            getThc(),
            2,
            &menu_48_pos, &menu_48_page, draw_menu, menu_48_click);
}
#endif

// Add after other menu definitions
int menu_49_pos = 0;
int menu_49_page = 0;
String menu_49_t[] = { "Input Shaper", "","Type", "None", "Freq Hz", "0", "Dampen", "0" };
int shaperVal[3];
const char* stypes[] = {"None", "ZV", "ZVD", "MZV", "EI"};

int getShaper(void) {
  shaperVal[0] = shaper.shaperType;
  shaperVal[1] = shaper.naturalFrequency * 100;
  shaperVal[2] = shaper.dampingRatio * 100;
  
  menu_49_t[3] = stypes[shaperVal[0]];
  menu_49_t[5] = String(shaperVal[1]*0.001f);
  menu_49_t[7] = String(shaperVal[2]*0.001f);
  return 9;
}

void menu_49_click(int a) {
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2)
    LOADMENU(4);
  if (a == IRK_OK || a == IRK4_OK) {
    // Save config and return to menu 4
    
    shaper.configure(shaperVal[0],shaperVal[1]*0.001,shaperVal[2]*0.001);
    LOADMENU(4);
  }
  
  int v = menu_setvalue(a);
  if (v != 0) {
    if (*menu_pos == 0) {
      // Type selection
      shaperVal[0] += (v > 0) ? 1 : -1;
      if (shaperVal[0] < 0) shaperVal[0] = 6;
      if (shaperVal[0] > 6) shaperVal[0] = 0;
      menu_49_t[3] = stypes[shaperVal[0]];
    } else if (*menu_pos == 1) {
      // Frequency adjustment
      shaperVal[1] += v;
      if (shaperVal[1] < 1) shaperVal[1] = 1;
      if (shaperVal[1] > 100) shaperVal[1] = 100;
      menu_49_t[5] = String(shaperVal[1] / 100.0f);
    } else if (*menu_pos == 2) {
      // Damping adjustment
      shaperVal[2] += v;
      if (shaperVal[2] < 1) shaperVal[2] = 1;
      if (shaperVal[2] > 100) shaperVal[2] = 100;
      menu_49_t[7] = String(shaperVal[2] / 100.0f);
    }
    draw_menu();
  }
}

void LOADMENU49(void) {
  col2pos = 14;
  load_menu(49, menu_49_t,
            getShaper(),
            2,
            &menu_49_pos, &menu_49_page, draw_menu, menu_49_click);
}

void menu_4_click(int a)
{
  static int menu4L=LEN(menu_4_t)-2;
  extern bool RPM_PID_MODE;
  vamul=1;
  if (a == IRK_LF  || a == IRK4_LF || a == IRK4_BACK2)
    LOADMENU(0);
  else if (a == IRK_OK || a == IRK4_OK) 
	{
		switch (*menu_pos) {
		  case 0:
			LOADMENU41();
			break;
		  case 1:
			LOADMENU42();
			break;
		  case 2:
			LOADMENU43();
			break;
		  case 3:
			LOADMENU44();
			break;
		  case 4:
			LOADMENU45();
			break;
		  case 5:
			LOADMENU46();
			break;
		  case 6:
			LOADMENU47();
			break;
		  case 7:
			LOADMENU48();
			break;
      case 8:
      LOADMENU49();
      break;
      case 9:
			//reload_eeprom();
      saveconfigs();
			LOADMENU(0);
      }    
	}
}
/*
 *  * ============================================================================================
*/

/*
    MENU 2, job list

*/
// Jobs menu, display files

#define NUMJOBS 50


int menu_2_pos = 0;
int menu_2_page = 0;
String menu_2_t[(NUMJOBS+1) * 2];

// #include <SPIFFS.h>
String ojobname;
void check_job() {
  if (ojobname != jobname) {
    dummy_beginuncompress("/" + jobname);
    ojobname = jobname;
  }
}

void clearJobs(void){
}
int getJobs(void)
{
  #ifdef ESP32
  File dir = SPIFFS.open("/");
  menu_2_t[0] = "File name";
  menu_2_t[1] = "Kb";
  ojobname = "";
  int i = 0;
  File file;
  while (file=dir.openNextFile()) {
    if (!file.isDirectory()){
      String s = file.name();
      if (s.endsWith(".gcode") && i<NUMJOBS) {
        //s.remove(0, 1); // littlefs fix
        s.remove(s.length() - 6);
        //s.remove(0,1);
        menu_2_t[i * 2 + 2] = s;
        menu_2_t[i * 2 + 3] = String(file.size() / 1000);
        i++;
      }
    }
  }
  #else
  Dir dir = SPIFFS.openDir("/");
  menu_2_t[0] = "File name";
  menu_2_t[1] = "Kb";
  ojobname = "";
  int i = 0;
  while (dir.next()) {
    String s = dir.fileName();
    if (s.endsWith(".gcode") && i<NUMJOBS) {
      s.remove(0, 1); // littlefs fix
      s.remove(s.length() - 6);
      //s.remove(0,1);
      menu_2_t[i * 2 + 2] = s;
      menu_2_t[i * 2 + 3] = String(dir.fileSize() / 1000);
      i++;
    }
  }
  #endif
  return i * 2 + 2;
}



void draw_menu(void)
{
  extern bool inwaitbuffer;
  //if (inwaitbuffer)return;

  d_clear(); // clear the internal memory
  d_setcolor(WHITE);
  d_text(0, 0, *menu_t); // write something to the internal memory
  d_rect(0, LCD_Y - 2, LCD_X * d_w(), 1);
  d_frect(0, LCD_Y * (1 + *menu_pos - *menu_page) - 1, LCD_X * d_w() + 2, LCD_Y);
  if (menu_col == 2) {
    d_text(20 - d_t_width(*(menu_t + 1)) / LCD_X, 0, *(menu_t + 1)); // write something to the internal memory
    d_rect(col2pos * LCD_X - 1, 0, 1, LCD_Y);
    d_setcolor(BLACK);
    d_rect(col2pos * LCD_X - 1, LCD_Y * (1 + *menu_pos - *menu_page), 1, LCD_Y + 2);
  }
  for (int i = 0; i < LINES - 1; i++) {
    int j = i + *menu_page;
    if (j > menu_page_le)
      break;
    if (lcd_kind==KIND_LCD2004){
      d_text(0, i + 1,j == *menu_pos ? ">" : " ");
    } else d_setcolor(j == *menu_pos ? BLACK : WHITE);
    d_text(1, i + 1, *(menu_t + j * menu_col + menu_col)); // write something to the internal memory
    if (menu_col == 2) {
      String* kb = (menu_t + j * menu_col + menu_col + 1);
      d_text(20 - d_t_width(*kb) / LCD_X, i + 1, *kb);
    }
  }
  d_show();
}

int menu_81_pos = 0;
int menu_81_page = 0;
String menu_81_t[] = { "", "Execute", "Resume", "Preview Area", "Delete", "Back" };

void menu_81_click(int a)
{
  if (a == IRK_OK || a == IRK4_OK || a == IRK_LF || a == IRK4_LF || a == IRK4_BACK2) {
    confirm_click((a == IRK_OK || a == IRK4_OK) ? *menu_pos : -1);
  }
}
#define LOADCONFIRM1(T, clickok) \
  menu_81_t[0] = T;            \
  confirm_click = clickok;     \
  LOADMENU(81);

int menu_82_pos = 0;
int menu_82_page = 0;
String menu_82_t[] = { "", "", "", "", "", "" };

void menu_82_click(int a)
{
  if (a == IRK_OK || a == IRK4_OK || a == IRK_LF || a == IRK4_LF || a == IRK4_BACK2) {
    confirm_click((a == IRK_OK || a == IRK4_OK) ? *menu_pos : -1);
  }
}
#define LOADCUSTOM(T, L1, L2, L3, L4, L5, clickok) \
  menu_82_t[0] = T;                              \
  menu_82_t[1] = L1;                             \
  menu_82_t[2] = L2;                             \
  menu_82_t[3] = L3;                             \
  menu_82_t[4] = L4;                             \
  menu_82_t[5] = L5;                             \
  confirm_click = clickok;                       \
  LOADMENU(82);

extern int uncompress;

void LOADMENU2FAIL(int a)
{
  if (a) {
    extern void stopmachine();
    stopmachine();
  }
  LOADMENU(0);
}
void LOADMENU2(void)
{
  if (!uncompress) {
    load_menu(2, menu_2_t,
              getJobs(),
              2,
              &menu_2_pos, &menu_2_page, draw_menu, menu_2_click);
  }
  else {
    LOADCONFIRM(("Stop running job !"), LOADMENU2FAIL); // CONFIRMATION MENU
  }
}

void menu_99_click(int a)
{
  if (a == IRK_OK || a == IRK4_OK) {
    LOADMENU(0);
  }
}

extern void connectWifi(int ret = 1);

extern int ISWIFIOK;
void RESTARTESP(int a)
{
  if (a) {
#ifdef ESP8266
    //eepromwrite(EE_softreset, 111);
    //eepromcommit();
    //delay(200);
    if (ISWIFIOK) {
      connectWifi();
      LOADINFO();
    } else ESP.restart();
#endif
  }
  else
    LOADMENU(0);
}
extern float OAX,OAY,OAZ;  

void menu_0_click(int a)
{
  if (a == IRK_LF || a == IRK4_LF || a == IRK4_BACK2)
    LOADINFO();

  if (a == IRK_OK || a == IRK4_OK) {
    switch (*menu_pos) {
      case 0:
        //for (int i = 0; i < strlen(gc92); i++)wifi_push(gc92[i]);
        extern void init_pos();
        OAX=OAY=OAZ=cx1=cy1=cz1=ocz1=0;
        init_pos();
        LOADINFO();
        break; // return to info menu
      case 1:
        LOADMENU2();
        break; // load the jobs menu
      case 2:
        LOADMENU(3);
        break; // settings menu
      case 3:
        LOADMENU(4);
        break; // settings menu
      case 4:
        if (ISWIFIOK)
          RESTARTESP(1);
        else
        {
          LOADCONFIRM(("Restart CNC ?"), RESTARTESP);
        }
        break; // settings menu
    }
  }
}

void DELETEJOB(int a)
{
  if (a) {
    deletejob("/" + jobname);
  }
  LOADMENU2();
}
void DUMMYJOB(int a)
{
  LOADMENU2();
}



void runjob(int a);
long lastpreview = 0;
void runpreview()
{
  //
  if (millis() - lastpreview < 5000) return;
  lastpreview = millis();
  LOADMENU2();
  extern int laserOn, constantlaserVal;
  laserOn = 1;
  constantlaserVal = 255;
  extern float ocz1, cx1, cy1;
  float rx = cx1;
  float ry = cy1;
  float rz = ocz1;
  set_tool(255);
  addmove(100, xMin, yMin, 2, 0, 1, 1); // ke kiri atas
  addmove(100, 0, 0, 0.5, 0, 0, 1);

  float dx=(xMax-xMin)/2;
  float dy=(yMax-yMin)/2;
    
  addmove(100, dx, 0, 0, 0, 1, 1); // ke kanan atas
  addmove(100, dx, 0, 0, 0, 1, 1); // ke kanan atas

  addmove(100, 0, 0, -0.5, 0, 0, 1);

  addmove(100, 0, dy, 0, 0, 1, 1); // kanan bawah
  addmove(100, 0, dy, 0, 0, 1, 1); // kanan bawah

  addmove(100, 0, 0, 0.5, 0, 0, 1);

  addmove(100, -dx, 0, 0, 0, 1, 1); // ke kanan atas
  addmove(100, -dx, 0, 0, 0, 1, 1); // ke kanan atas
  
  addmove(100, 0, 0, -0.5, 0, 0, 1);

  //addmove(100, 0, yMin-yMax, 0, 0, 1, 1);
  //addmove(100, 0, 0, 0.5, 0, 0, 1);
  addmove(100, 0, -dy, 0, 0, 1, 1); // kanan bawah
  addmove(100, 0, -dy, 0, 0, 1, 1); // kanan bawah

  //addmove(100, 0, 0, -2.5, 0, 1, 1);
  laserOn = 0;
  addmove(100, rx, ry, ocz1, 0, 1, 0);
  addmove(100, rx, ry, rz, 0, 1, 0);
  waitbufferempty();
  set_tool(0);

}

void PREVIEWJOB(int a)
{
  if (a == 4) {
    runpreview();
  }
  else if (a == -1) {
    LOADCONFIRM1((jobname), runjob);
  }
  else
    LOADINFO();
}

String menu_91_t[] = { "Obj", "Obj", "Obj", "0","Exec","All"}; 
int menu_91_pos=0;
int menu_91_page=0;
int exec_all=1;
int rpos;
extern int shapes_ctr;
int checktm1;
void updaterpos(){
    menu_91_t[3] = rpos;
    menu_91_t[2] = String(shapes[rpos].px+OAX)+","+String(shapes[rpos].py+OAY);
    // goto shape position
    float pz=shapes[rpos].pz;
    addmove(100, 0,0,4, 0, 1, 1);
    addmove(100, shapes[rpos].px+OAX, shapes[rpos].py+OAY, ocz1, 0, 1, 0);
    addmove(100, 0,0,-4, 0, 1, 1);
    if (exec_all==0) menu_91_t[5]="Single";
    if (exec_all==1) menu_91_t[5]="All";
}

void menu_91_exit(){
    addmove(100, 0,0,10, 0, 1, 1);
    addmove(100, OAX, OAY, ocz1, 0, 1, 0);
    addmove(100, 0,0,-10, 0, 1, 1);  
}
extern void special_loop(int xcmd,bool skipcheck);
void menu_91_click(int a)
{
  if (a == IRK_H || a == IRK4_H || a == IRK4_BACK2)
    LOADMENU2();
  if (a == IRK_OK || a == IRK4_OK) {
    if (millis()-checktm1>3000) {
      // resume job from here
      extern bool singleobj;
      singleobj=exec_all==0;
      beginuncompress("/" + jobname,true,rpos);
      menu_exit=default_exit;
      LOADMENU(0);
      LOADINFO();
    }
  }
  //vamul=1;
  int v = 0;
  switch (a) {
      case IRK_4:
      case IRK4_4:
      case IRK_2:
      case IRK4_2:
      case IRK_6:
      case IRK4_6:
      case IRK_8:
      case IRK4_8:
      case IRK_3:
      case IRK4_3:
      case IRK_9:
      case IRK4_9:
          special_loop(a,true);
          break;
      default:
          v = menu_setvalue(a);
          break;
  }
  if (v != 0) {
    if (menu_91_pos==0){  
        rpos += v;
        if (rpos<0)rpos=0;
        if (rpos>shapes_ctr-1) rpos=shapes_ctr-1;
    } else if (menu_91_pos==1){  
        exec_all=(exec_all+1)&1;
    }
    updaterpos();
    draw_menu();
  }
}

void runjob(int a)
{
  if (a == 0) {
    // jobname
    //dummy_beginuncompress("/" + jobname);

    
    beginuncompress("/" + jobname,false,0);
    LOADMENU(0);
    LOADINFO();
  }
  else if (a == 1) {
//      #ifdef PREMIUMFEATURE
      dummy_beginuncompress("/" + jobname);
      rpos=0;
      updaterpos();
      menu_91_t[0]=String(shapes_ctr)+" Obj";
      checktm1=millis();
      load_menu(91, menu_91_t,
            6,2,
            &menu_91_pos, &menu_91_page, draw_menu, menu_91_click);
      menu_exit=menu_91_exit;
//      #else
//      LOADCONFIRM(("Not available"), DUMMYJOB)
//      #endif
  }
  else if (a == 2) {
    // jobname

    check_job();
    extern int uctr;
    
    LOADCUSTOM("Job /" + jobname,
               "Area X Y Z cm",
               String((xMax-xMin) * 0.1) + "  " + String((yMax-yMin) * 0.1) + "  " + String(zMin * -0.1),
               "F-avg " + String(dMax/tMax) + "mm/s",
               "Time " + String(uptime(2000 * tMax)),

               "Machine Preview",
               PREVIEWJOB);
               
  }
  else if (a == 3) {
    // jobname
    LOADCONFIRM(("Confirm delete ?"), DELETEJOB);
  } // run jobs
  else
    LOADMENU2();
}
void menu_2_click(int a)
{
  if (a == IRK_LF || a == IRK4_LF || a == IRK4_BACK2) // back to main menu
    LOADMENU(0);
  if (a == IRK_OK || a == IRK4_OK) {
    jobname = menu_2_t[(*menu_pos) * 2 + 2] + ".gcode";
    LOADCONFIRM1((jobname), runjob); // CONFIRMATION MENU
  }
  if (a == IRK4_REVEAL) {
    jobname = menu_2_t[(*menu_pos) * 2 + 2] + ".gcode";
    check_job();
    runpreview();
  }
  
  float x, y, z;
  x = 0;
  y = 0;
  z = 0;
  switch (a) {
    case IRK_1: case IRK4_1: z = 1; break;
    case IRK_7: case IRK4_7: z = -1; break;
    case IRK_3: case IRK4_3: z = 0.5; break;
    case IRK_9: case IRK4_9: z = -0.5; break;
    case IRK_2: case IRK4_2: y = -1; break;
    case IRK_4: case IRK4_4: x = -1; break;
    case IRK_6: case IRK4_6: x = 1; break;
    case IRK_8: case IRK4_8: y = 1; break;
    default: return;
  }
  
  if (x != 0 || y != 0 || z != 0) {
    addmove(100, x, y, z, 0, 1, 1);
  }
  
}

void menu_3_click(int a)
{
  if (a == IRK_LF || a == IRK4_LF || a == IRK4_BACK2)
    menu_2_click(a);
  if (!(a == IRK_OK || a == IRK4_OK))
    return;
  switch (*menu_pos) {
    case 0:
      if ((ax_home[0] < 1) && (ax_home[1] < 1) && (ax_home[2] < 1)) {
        addmove(100, 0, 0, 10, 0, 1, 1);
        addmove(100, 0, 0, ocz1, 0, 1, 0);
        addmove(100, 0, 0, 0, 0, 1, 0);
      }
      else {
        homing();
        update_pos();
      }
      LOADINFO();
      break;
    case 1:
      addmove(100, 0, 0, 10, 0, 1, 1);
      addmove(100, OAX, OAY, ocz1, 0, 1, 0);
      addmove(100, OAX, OAY, OAZ, 0, 1, 0);
      
      LOADINFO();
      break;
  }
}


#include <WiFiClient.h>
extern IPAddress ip;


int xlaserOn, xconstantlaserVal;
long lastpause;
extern int8_t PAUSE;
uint32_t autoresume,zpausestep,zpausedir;
extern int uncompress;

void dopause(int tm,bool stopping) {

  if (uncompress) {
    ispause = ispause == 0 ? 1 : 0;
    autoresume=0;
    extern int cmdlaserval;    
    extern bool toolWasOn;
   if (ispause) {
      PAUSE = ispause;

      //xlaserOn = toolWasOn;
      //xconstantlaserVal = cmdlaserval;
      
      waitbufferempty(false);
      // dx
      zpausestep=tm==0?(info_z-OAZ)*fabs(stepmmx[2]):0;
      if (uncompress && lasermode==0 && !stopping){
        // bring up the tool to 0
        motor2steps(2,zpausestep);
      }
      if (tm>0)autoresume=millis()+tm;
    } else {
      if (uncompress && lasermode==0 && !stopping){
        // bring up the tool to old position
        motor2steps(2,-zpausestep);
        zpausestep=0;
      }        
      extern int machinefail;  
      machinefail=10;
      //BuzzError(LOW);  
 
      extern bool FULLSPEED;
      FULLSPEED=false;//cmhead==cmtail;
      PAUSE = ispause;      
      //toolWasOn = xlaserOn;
      //cmdlaserval = xconstantlaserVal;
    }
  }
  mili = 0;  
}

static int IR_UIloop(int icommand)
{
  // handle special keys
  
  extern float tf_multiplier;
  switch (icommand) {
    case IRK4_JOBINDEX:
      LOADMENU2();
      break;
    case IRK4_FASTER:
    
      if (menu_index==99 && tf_multiplier < 3)tf_multiplier += 0.25;
      mili = 0;
      break;
    case IRK4_SLOWDOWN:
      if (menu_index==99 && tf_multiplier > 0.3)tf_multiplier -= 0.25;
      mili = 0;
      break;
    case IRK4_PLAY:
      if (ispause) {
        dopause();
      }

      break;
    case IRK4_EXIT:
      LOADINFO();
      break;
    case IRK4_BACK:
      icommand = IRK4_LF;
      break;
    case IRK4_PAUSE:
      dopause();
      break;
    case IRK4_SETTINGS:
      LOADMENU(4);
      break;
    case IRK4_POWER:
      //restartLCD();
      extern void load_info();
      load_info();
      xdisplay.Reset();
      
      break;
    case IRK4_STOP:
      extern void stopmachine();
      dopause(0,true);
      stopmachine();
      ispause=PAUSE=0;
      break;
    case IRK4_SPINDLEUP:
    case IRK4_SPINDLEDN:
      if (lasermode==1) {
        extern void testLaser(void);
        testLaser();
      } else {
        set_tool(lastS +(icommand==IRK4_SPINDLEDN? -25:25) );
        mili = 0;
      }
      break;
    case IRK4_DOT1:
      tmul = 25;
      break;
    case IRK4_DOT2:
      tmul = 100;
      break;
    case IRK4_DOT3:
      tmul = 250;
      break;
    case IRK4_DOT4:
      tmul = 500;
      break;
  }

  if (menu_index == 99 && !(icommand == IRK_OK || icommand == IRK4_OK)) {
    return 0;
  }
  int y = 0;
  // jog on arrow key = small incremental
  if (icommand == IRK_UP || icommand == IRK4_UP)
    y = -1;
  if (icommand == IRK_DN || icommand == IRK4_DN)
    y = +1;
  else {
    //display.init();
    menu_click(icommand);
  }
  if (menu_page_le < 0)
    return 1;
  if (y != 0) {
    *menu_pos += y;
    int le = menu_page_le;
    if (*menu_pos < 0) {
      *menu_pos = le;
      *menu_page = max(0, le - (LINES - 2));
    }
    if (*menu_pos > le) {
      *menu_pos = 0;
      *menu_page = 0;
    }
    if (*menu_pos > (*menu_page) + (LINES - 2))
      (*menu_page)++;
    if (*menu_pos < *menu_page)
      (*menu_page)--;

    menu_f();
  }
  return 1;
}

int ir_oled_loop(int icommand)
{

  if (icommand)rmkey = icommand;
  extern int8_t RUNNING;
  extern uint32_t cm;

  
  if (autoresume>0 && ispause && millis()>autoresume){
      dopause();
      return 0;
  } // resume
  
  // update lcd every 30ms if nothing pressed
#ifdef ESP32  
  if (icommand == 0 && ((cm - mili) > (RUNNING?100000:30000)) ) {
#else
  if (icommand == 0 && ((cm - mili) > (RUNNING?1000000:300000)) ) {
#endif
    mili = cm;
    if (menu_index == 99) menu_f();
    return 0;
  }
  if (icommand)mili = cm;
  return IR_UIloop(icommand);
}

void load_info() {
  menu_exit = default_exit;
  LOADINFO();
}




void restartLCD(){
  IR_end();
  oledready = true;
  #if INCLUDE_I2CLCD
  if (lcd_kind==KIND_LCD2004){
    //LCD.begin(lcd_sda,lcd_scl,0x27,20,4);
    LCD.begin(lcd_sda,lcd_scl,0x27,20,4);
    LCD.setBacklight(100);
    oldindex=-1;
    LCD_Y=1;
    LCD_X=1;
    YOFS=0;
    XOFS=0;
    LINES=4;    
  } else 
    #endif
    {
    xdisplay.setparam(lcd_kind,lcd_addr,lcd_sda,lcd_sda2, lcd_scl, 255,lcd_rst); 
    xdisplay.setFont((lcd_kind==KIND_NK1661 || lcd_kind==KIND_ST7735)?BIGFONT:BASICFONT);
    xdisplay.init();
    if (lcd_kind==KIND_NK1202){
      LCD_Y=14;
      LCD_X=4.8;
      YOFS=1;
      XOFS=1;
      LINES=5;  
    }
    if (lcd_kind==KIND_ST7565){
      LCD_Y=13.2;
      LCD_X=6.4;
      YOFS=0;
      XOFS=0;
      LINES=5;  
    }
    if (lcd_kind==KIND_NK6100){
      LCD_Y=20;
      LCD_X=6.2;
      YOFS=1;
      XOFS=1;
      LINES=6;  
    }
    if (lcd_kind==KIND_ST7735){
      LCD_Y=21;
      LCD_X=7.75;
      YOFS=1;
      XOFS=1;
      LINES=6;  
    }
    if (lcd_kind==KIND_NK1661){
      LCD_Y=21;
      LCD_X=7.75;
      YOFS=1;
      XOFS=1;
      LINES=6;  
    }
    if (lcd_kind==KIND_OLED1306){
      LCD_Y=16;
      LCD_X=6.5;
      YOFS=0;
      XOFS=0;
      LINES=4;  
    }  
  }
  
  mili = 0;
  //+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]);
  LOADINFO();
  IR_setup();
  
  
}

void d_rect(int a,int b,int c,int d) {
  #if INCLUDE_I2CLCD    
  if (lcd_kind==KIND_LCD2004) return;
  #endif
  xdisplay.drawRect(a,b,c,d);
}
void d_frect(int a,int b,int c,int d) {
  #if INCLUDE_I2CLCD    
  if (lcd_kind==KIND_LCD2004) return;
  #endif
  xdisplay.fillRect(a,b,c,d);
}

void d_clear() {
  #if INCLUDE_I2CLCD
  if (lcd_kind==KIND_LCD2004){
    LCD.home();
    LCD.clear();
  } else 
  #endif
  xdisplay.clear();
}
void d_show() { 
  #if INCLUDE_I2CLCD
  if (lcd_kind==KIND_LCD2004) return;
  #endif
  IR_end();
  xdisplay.display();
  IR_setup();
}
void d_setcolor(OLEDDISPLAY_COLOR x) {
  #if INCLUDE_I2CLCD
  if (lcd_kind==KIND_LCD2004) return;
  #endif
  xdisplay.setColor(x);
}
void  d_text(int x,int y,String s) {
  #if INCLUDE_I2CLCD
  if (lcd_kind==KIND_LCD2004) {
    LCD.setCursor(x,y);
    LCD.print(s);
  } else 
  #endif
  {
    xdisplay.drawString((x)*LCD_X+XOFS,(y)*LCD_Y+YOFS,s);
  }
}
int d_t_width(String s) {
  #if INCLUDE_I2CLCD
  if (lcd_kind==KIND_LCD2004) return s.length();
  #endif
  return xdisplay.getStringWidth(s);
}
