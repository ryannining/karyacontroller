#include "config_pins.h"
#ifdef WIFISERVER
#include <ESP8266WiFi.h>
#endif

#if defined(IR_OLED_MENU)

static int wait_job = 0;
static int wait_spindle = 0;
static String jobnum;

#include "ir_remote.h"
#include "ir_oled.h"

//#define OLEDDISPLAY_REDUCE_MEMORY

#ifndef LCD_SDA
#define LCD_SDA TX
#define LCD_SCL RX
#define LCD_CMD D1
#endif

#ifdef LCD_OLED
#include "SH1106Wire.h"
SH1106Wire xdisplay(0x3c, LCD_SDA, LCD_SCL); // d6,d5
#endif

#ifdef LCD_OLED_SSD
#include "SSD1306Wire.h"
SSD1306Wire xdisplay(0x3c, LCD_SDA, LCD_SCL); // d6,d5
#endif

#ifdef LCD_UC1609
#include "UC1609Wire.h"
UC1609Wire xdisplay(LCD_SDA, LCD_SCL, LCD_CMD);
#endif

#ifdef LCD_NK1202
#include "NK1202Wire.h"

NK1202Wire xdisplay(LCD_SDA, LCD_SCL,HAS_CS);


#endif

#ifdef LCD_NK1661
#define USERGB12
#include "NK1661Wire.h"
//#define HAS_CS D2

NK1661Wire xdisplay(LCD_SDA, LCD_SCL, HAS_CS, 20); // 

#endif

#ifdef LCD_2004
#include "LiquidCrystal_PCF8574.h"
#include <Wire.h> // Only needed for Arduino 1.6.5 and earlier
LiquidCrystal_PCF8574 xdisplay(IR_OLED_MENU); // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

#include "fonts.h"
#include "gcodesave.h"
#include "gcode.h"
#include "motion.h"
#include "eprom.h"
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
String IPA;
int* menu_pos;
int* menu_page;
int menu_col;
typedef void (*Tdrawmenu)(void);
typedef void (*Tclickmenu)(int);
Tdrawmenu menu_f;
Tclickmenu menu_click;
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
    f();
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
int tmul = 100;
int rmkey;
extern long lastjobt;
extern int spindle_pct, lastS;
extern long spindle_pwm;
uint8_t LCDturn;
void draw_info(void)
{
    extern bool inwaitbuffer;
    if (inwaitbuffer)return;
    
    d_clear(); // clear the internal memory
    d_setcolor(WHITE);
    extern int ISWIFIOK;

    #ifdef WIFISERVER
    if (ISWIFIOK && (WiFi.status() != WL_CONNECTED))
        d_text(0, 0, "Wifi disconnect"); // write something to the internal memory
    else
        d_text(0, 0, IPA); // write something to the internal memory
    #else
        d_text(0, 0, "Karyacontroller"); // write something to the internal memory
    #endif
    
    d_frect(0, LCD_Y - 1, d_w() * LCD_X, LCD_Y);
    d_setcolor(BLACK);
    d_text(1, 1, String(info_x)); // write something to the internal memory
    d_text(8, 1, String(info_y)); // write something to the internal memory
    d_text(15, 1, String(info_z)); // write something to the internal memory
    d_setcolor(WHITE);

#ifdef RPM_COUNTER
    extern uint32_t get_RPM();
    uint32_t r = get_RPM();
    int pw;
    extern bool RPM_PID_MODE;
    if (RPM_PID_MODE) {
        if (avgcnt > 0) {
            pw = avgpw / avgcnt;
            avgpw = 0;
            avgcnt = 1;
        }
    }
    else
        pw = spindle_pct;

    d_text(0, 2, "SPD  " + String(r) + "/" + String(pw) + "%"); // write something to the internal memory
#else
#define r spindle_pct
    d_text(0, 2, "SPD% " + String(r)); // write something to the internal memory
    d_text(10, 2, "RT% " + String(int(f_multiplier*100))); // write something to the internal memory
#endif
    String L1, L2, L3;
    if (uncompress) {
        if (PAUSE)L1="Paused !"; else
        L1 = jobname;
        
        L1 += " "+String(100 * gcodepos / gcodesize) + "%";
    }
    else {
        L1 = "JOG% " + String(tmul);
    }
    L2 = "Uptime " + uptime(millis());
    L3 = "Last " + (lastjobt > 0 ? uptime(lastjobt) : "-");

    if (LINES == 4) {
        LCDturn++;
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
        LCDturn++;
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
String menu_3_t[] = { "Tools",  "1. Preview","2. Laser Test","3. Homing" };

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
String menu_4_t[] = { "Settings", "1. Speed", "2. Acceleration", "3. Backlash", "4. Step/mm", "5. Homing", "Save to EEPROM", "Update firmware" };

// Standar menu is index 0: title, rest is the menu
int menu_41_pos = 0;
int menu_41_page = 0;
String menu_41_t[] = { "Max Speed", "mm/s", "Axis X", "50", "Axis Y", "50", "Axis Z", "5", "Spindle %", "1" };
int xyzspeed[4] = { 10, 20, 30, 1 };

int getSpeed(void)
{
    xyzspeed[0] = maxf[MX];
    xyzspeed[1] = maxf[MY];
    xyzspeed[2] = maxf[MZ];
    xyzspeed[3] = Lscale * 100;

    menu_41_t[3] = xyzspeed[0];
    menu_41_t[5] = xyzspeed[1];
    menu_41_t[7] = xyzspeed[2];
    menu_41_t[9] = xyzspeed[3];
    return 10;
}

int menu_setvalue(int a)
{
    if (a == IRK_LF || a == IRK4_LF)
        return -1;
    if (a == IRK_RG || a == IRK4_RG)
        return 1;
    if (a == IRK_4 || a == IRK4_4)
        return -5;
    if (a == IRK_6 || a == IRK4_6)
        return 5;
    if (a == IRK_X || a == IRK4_X)
        return -1000;
    return 0;
}

void menu_41_click(int a)
{
    if (a == IRK_H || a == IRK4_H|| a == IRK4_BACK2)
        LOADMENU(4);
    if (a == IRK_OK || a == IRK4_OK) {
        // save config and back to menu 4
        maxf[MX] = xyzspeed[0];
        maxf[MY] = xyzspeed[1];
        maxf[MZ] = xyzspeed[2];
        Lscale = (float)xyzspeed[3] / 100.0;
        eepromwrite(EE_max_x_feedrate, xyzspeed[0]); // accell
        eepromwrite(EE_max_y_feedrate, xyzspeed[1]); // accell
        eepromwrite(EE_max_z_feedrate, xyzspeed[2]); // accell
        eepromwrite(EE_Lscale, ff(Lscale));

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
int xyzaccel[3] = { 8000, 220, 30 };

int getAccel(void)
{
    xyzaccel[0] = xyjerk;
    xyzaccel[1] = accel;
    xyzaccel[2] = xycorner;
    menu_42_t[3] = xyzaccel[0];
    menu_42_t[5] = xyzaccel[1];
    menu_42_t[7] = xyzaccel[2];
    return 8;
}

void menu_42_click(int a)
{
    if (a == IRK_H || a == IRK4_H|| a == IRK4_BACK2)
        LOADMENU(4);
    if (a == IRK_OK || a == IRK4_OK) {
        // save config and back to menu 4
        /*
      eprom_wr(51, EE_accel, S_I);
      eprom_wr(67, EE_jerk, S_I);
      eprom_wr(181, EE_corner, S_I);
    */
        xyjerk = xyzaccel[0];
        accel = xyzaccel[1];
        xycorner = xyzaccel[2];
        eepromwrite(EE_accel, xyzaccel[1]); // accell
        eepromwrite(EE_jerk, xyzaccel[0]); // accell
        eepromwrite(EE_corner, xyzaccel[2]); // accell
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
String menu_43_t[] = { "De-Backlash", "mm", "X", "0", "Y", "250", "Z", "15" };
int xyzback[3] = { 10, 10, 3 };

int getBacklash(void)
{
    xyzback[0] = xback[MX];
    xyzback[1] = xback[MY];
    xyzback[2] = xback[MZ];

    menu_43_t[3] = 0.001 * xyzback[0];
    menu_43_t[5] = 0.001 * xyzback[1];
    menu_43_t[7] = 0.001 * xyzback[2];
    col2pos = 14;
    return 8;
}

void menu_43_click(int a)
{
    if (a == IRK_H || a == IRK4_H|| a == IRK4_BACK2)
        LOADMENU(4);
    if (a == IRK_OK || a == IRK4_OK) {
        // save config and back to menu 4
        //EE_xbacklash
        xback[MX] = xyzback[0];
        xback[MY] = xyzback[1];
        xback[MZ] = xyzback[2];

        eepromwrite(EE_xbacklash, xyzback[0]); // *1000
        eepromwrite(EE_ybacklash, xyzback[1]); // *1000
        eepromwrite(EE_zbacklash, xyzback[2]); // *1000

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
String menu_44_t[] = { "1 mm =", "Step", "X", "0", "Y", "250", "Z", "15" };
int xyzsteps[3] = { 19512, 19512, 230400 };

int getSteps(void)
{
    xyzsteps[0] = stepmmx[0] * 1000;
    xyzsteps[1] = stepmmx[1] * 1000;
    xyzsteps[2] = stepmmx[2] * 1000;

    menu_44_t[3] = 0.001 * xyzsteps[0];
    menu_44_t[5] = 0.001 * xyzsteps[1];
    menu_44_t[7] = 0.001 * xyzsteps[2];
    return 8;
}

void menu_44_click(int a)
{
    if (a == IRK_H || a == IRK4_H|| a == IRK4_BACK2)
        LOADMENU(4);
    if (a == IRK_OK || a == IRK4_OK) {
        // save config and back to menu 4
        //EE_estepmm
        stepmmx[0] = xyzsteps[0] * 0.001;
        stepmmx[1] = xyzsteps[1] * 0.001;
        stepmmx[2] = xyzsteps[2] * 0.001;
        eepromwrite(EE_xstepmm, xyzsteps[0]);
        eepromwrite(EE_ystepmm, xyzsteps[1]);
        eepromwrite(EE_zstepmm, xyzsteps[2]);
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
int xyzhome[3] = { 19512, 19512, 230400 };

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
    if (a == IRK_H || a == IRK4_H|| a == IRK4_BACK2) {
        LOADMENU(4);
        reload_eeprom();
    }
    if (a == IRK_OK || a == IRK4_OK) {
        // save config and back to menu 4
        //EE_xhome
        ax_home[0] = xyzhome[0];
        ax_home[1] = xyzhome[1];
        ax_home[2] = xyzhome[2];
        eepromwrite(EE_xhome, xyzhome[0] * 1000);
        eepromwrite(EE_yhome, xyzhome[1] * 1000);
        eepromwrite(EE_zhome, xyzhome[2] * 1000);
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
void UPDATEESP(int a)
{
    if (a) {
        extern void updateFirmware();
        updateFirmware();
    }
    LOADMENU(0);
}
void menu_4_click(int a)
{
    extern bool RPM_PID_MODE;
    if (a == IRK_LF  || a == IRK4_LF || a == IRK4_BACK2)
        LOADMENU(0);
    else if (a == IRK_OK || a == IRK4_OK)
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
            eepromcommit();
            LOADMENU(0);
            break;
        case 6:
            LOADCONFIRM(("UPDATE FIRMWARE ?"), UPDATEESP);
        }
}
/*
 *  * ============================================================================================
*/

/*
    MENU 2, job list

*/
// Jobs menu, display files
#ifdef __ARM__
#define NUMJOBS 20
#endif
#ifdef ESP8266
#define NUMJOBS 50
#endif

int menu_2_pos = 0;
int menu_2_page = 0;
String menu_2_t[NUMJOBS * 2];

#ifdef ESP8266
#include <FS.h>
int getJobs(void)
{
    Dir dir = SPIFFS.openDir("/");
    menu_2_t[0] = "File name";
    menu_2_t[1] = "Kb";
    int i = 0;
    while (dir.next()) {
        String s = dir.fileName();
        if (s.endsWith(".gcode")) {
            s.remove(0, 1);
            s.remove(s.length() - 6);
            //s.remove(0,1);
            menu_2_t[i * 2 + 2] = s;
            menu_2_t[i * 2 + 3] = String(dir.fileSize() / 1000);
            i++;
        }
    }

    return i * 2 + 2;
}
#endif

#ifdef __ARM__
#include <SdFat.h>
int getJobs(void)
{
    Dir dir = SPIFFS.openDir("/");
    menu_2_t[0] = "File name";
    menu_2_t[1] = "Kb";
    int i = 0;
    while (dir.next()) {
        String s = dir.fileName();
        if (s.endsWith(".gcode")) {
            s.remove(0, 1);
            s.remove(s.length() - 6);
            //s.remove(0,1);
            menu_2_t[i * 2 + 2] = s;
            menu_2_t[i * 2 + 3] = String(dir.fileSize() / 1000);
            i++;
        }
    }

    return i * 2 + 2;
}
#endif

void draw_menu(void)
{
    extern bool inwaitbuffer;
    if (inwaitbuffer)return;

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

        d_setcolor(j == *menu_pos ? BLACK : WHITE);
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
String menu_81_t[] = { "", "Execute", "Preview Area", "Delete", "Back" };

void menu_81_click(int a)
{
    if (a == IRK_OK ||a == IRK4_OK || a == IRK_LF||a == IRK4_LF ||a==IRK4_BACK2) {
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
    if (a == IRK_OK ||a == IRK4_OK || a == IRK_LF||a == IRK4_LF ||a==IRK4_BACK2) {
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
        enduncompress();
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

extern void wifi_push(char c); // we use WIFI GCODE Command buffer to inject gcode
static const char* gc92 = "G92\n";
extern void connectWifi(int ret=1);
extern int ISWIFIOK;
void RESTARTESP(int a)
{
    if (a) {
    #ifdef ESP8266
        //eepromwrite(EE_softreset, 111);
        //eepromcommit();
        //delay(200);
        if (ISWIFIOK){
            connectWifi();
            LOADINFO();
        } else ESP.restart();
    #endif
    }
    else
        LOADMENU(0);
}
void menu_0_click(int a)
{
    if (a == IRK_LF || a == IRK4_LF || a == IRK4_BACK2)
        LOADINFO();

    if (a == IRK_OK || a==IRK4_OK) {
        switch (*menu_pos) {
        case 0:
            for (int i = 0; i < strlen(gc92); i++)
                wifi_push(gc92[i]);
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
void PREVIEWJOB(int a)
{
    if (a == 4) {
        LOADINFO();
extern int laserOn,constantlaserVal;
        laserOn=1;
        constantlaserVal=255;
        
        set_pwm(150);
        addmove(100, 0, 0, 2, 0, 1, 1);
        addmove(100, xMax - xMin, 0, 0, 0, 1, 1);

        addmove(100, 0, 0, 0.5, 0, 0, 1);

        addmove(100, 0, yMax - yMin, 0, 0, 1, 1);
        addmove(100, 0, 0, -0.5, 0, 0, 1);

        addmove(100, xMin - xMax, 0, 0, 0, 1, 1);
        addmove(100, 0, 0, 0.5, 0, 0, 1);

        addmove(100, 0, yMin - yMax, 0, 0, 1, 1);
        addmove(100, 0, 0, -0.5, 0, 0, 1);

        addmove(100, 0, 0, -2, 0, 1, 1);
        laserOn=0;
        constantlaserVal=255;
        //waitbufferempty();
        //set_pwm(0);
    }
    else if (a == -1) {
        LOADCONFIRM1((jobname), runjob);
    }
    else
        LOADINFO();
}
void runjob(int a)
{
    if (a == 0) {
        // jobname
        beginuncompress("/" + jobname);
        LOADINFO();
    }
    else if (a == 1) {
        // jobname

        dummy_beginuncompress("/" + jobname);

        LOADCUSTOM("Job /" + jobname,
            "Area X Y Z cm",
            String((xMax - xMin) * 0.1) + "  " + String((yMax - yMin) * 0.1) + "  " + String(zMin * -0.1),
            "F-avg " + String(dMax / tMax) + "mm/s",
            "Time " + String(uptime(2000 * tMax)),

            "Machine Preview",
            PREVIEWJOB);
    }
    else if (a == 2) {
        // jobname
        LOADCONFIRM(("Confirm delete ?"), DELETEJOB);
    } // run jobs
    else
        LOADMENU2();
}
void menu_2_click(int a)
{
    if (a == IRK_LF || a == IRK4_LF||a == IRK4_BACK2) // back to main menu
        LOADMENU(0);
    if (a == IRK_OK || a == IRK4_OK) {
        jobname = menu_2_t[(*menu_pos) * 2 + 2] + ".gcode";
        LOADCONFIRM1((jobname), runjob); // CONFIRMATION MENU
    }
}

void menu_3_click(int a)
{
    if (a == IRK_LF|| a == IRK4_LF || a == IRK4_BACK2)
        menu_2_click(a);
    if (!(a == IRK_OK || a==IRK4_OK))
        return;
    switch (*menu_pos) {
    case 0:
        beginuncompress("/preview.gcode");
        LOADINFO();                  
        break;
    case 1:
        extern void testLaser(void);
        testLaser();
        break;
    case 2:
        if ((ax_home[0] == 0) && (ax_home[1] == 0) && (ax_home[2] == 0)) {
            addmove(100, 0, 0, 10, 0, 1, 1);
            addmove(100, 0, 0, 10, 0, 1, 0);
            addmove(100, 0, 0, 0, 0, 1, 0);
        }
        else {
            homing();
            update_pos();
        }
        LOADINFO();
        break;
    }
}

long mili;
#include <WiFiClient.h>
extern IPAddress ip;
bool oledready = false;
void setup_oled(void)
{
    //oledready=eepromread(EE_softreset)==111;
    //rst_info *resetInfo;
    //resetInfo = ESP.getResetInfoPtr();
    //tmul=ESP.getResetReason().toInt();
    //oledready=tmul!=0;
    if (!oledready)
        INITDISPLAY

    mili = millis();

    //+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]);
    LOADINFO();
    oledready = true;
}

void dopause(){
    if (uncompress){
        ispause = ispause == 0 ? 1 : 0;
        extern int8_t PAUSE;
        PAUSE = ispause;
    }
}
static int IR_UIloop(int icommand)
{
    // handle special keys
    extern float f_multiplier;
    switch (icommand){
        case IRK4_JOBINDEX:
            LOADMENU2();
            break;
        case IRK4_FASTER:
            f_multiplier=1.5;
            break;
        case IRK4_SLOWDOWN:
            f_multiplier=0.666;
            break;
        case IRK4_PLAY:
            if (ispause){
                dopause();
            } else f_multiplier=1;
            break;
        case IRK4_EXIT:
            LOADINFO();
            break;
        case IRK4_BACK:
            icommand=IRK4_LF;
            break;
        case IRK4_PAUSE:
            dopause();
            break;
        case IRK4_SETTINGS:
            LOADMENU(4);
            break;
        case IRK4_POWER:
            IR_end();
            REINIT;
            IR_setup();
            break;
        case IRK4_STOP:
            extern void stopmachine();
            stopmachine();
            enduncompress();
            break;
        case IRK4_SPINDLEDN:
            #ifdef laser_pin
            extern void testLaser(void);
            testLaser();
            #else
            set_pwm(lastS-25);
            #endif
            break;
        case IRK4_SPINDLEUP:
            #ifdef laser_pin
            extern void testLaser(void);
            testLaser();
            #else
            set_pwm(lastS+25);
            #endif
            break;
        case IRK4_DOT1:
            tmul=25;
            break;
        case IRK4_DOT2:
            tmul=100;
            break;
        case IRK4_DOT3:
            tmul=250;
            break;
        case IRK4_DOT4:
            tmul=500;
            break;
    }

    if (menu_index == 99 && !(icommand == IRK_OK || icommand == IRK4_OK)){
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
    if (icommand)rmkey=icommand;
    if (icommand == 0 && (millis() - mili > (uncompress ? 2500 : 1000))) {
        mili = millis();
        if (menu_index == 99)
            menu_f();
    }
    if (icommand)mili = millis();

    return IR_UIloop(icommand);
}
#else
#endif
