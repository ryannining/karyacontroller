#include "config_pins.h"

#if defined( IR_OLED_MENU )
#define xfont BASICFONT
#define YOFS 0
#define XOFS 0
#define LINES 4


#ifdef LCD_OLED
#include "SH1106Wire.h"
extern SH1106Wire xdisplay;
#define LCD_Y 16
#define LCD_X 6.4
#define YOFS 1
#endif

#ifdef LCD_OLED_SSD
#include "SSD1306Wire.h"
extern SSD1306Wire xdisplay;
#define LCD_Y 16
#define LCD_X 6.4
#define YOFS 1
#endif


#ifdef LCD_UC1609
#include "UC1609Wire.h"
extern UC1609Wire xdisplay;
#define LCD_Y 16
#define LCD_X 9.6
#define xfont BASICFONT
#endif

#ifdef LCD_NK1202
#include "NK1202Wire.h"
extern NK1202Wire xdisplay;
#define LINES 5
#define LCD_Y 14
#define LCD_X 4.8
#define YOFS 0
#endif

#ifdef LCD_NK1661
#define USERGB12
#include "NK1661Wire.h"
extern NK1661Wire xdisplay;
#define LCD_Y 21
#define LCD_X 7.75
#define YOFS 0
#define XOFS 1
#define xfont BIGFONT
#define LINES 6
#endif


#define REINIT xdisplay.Reset();



extern void IR_setup();

#ifdef LCD_2004
#include "LiquidCrystal_PCF8574.h"
extern LiquidCrystal_PCF8574 xdisplay;
#define LCD_Y 1
#define LCD_X 1
#endif

#ifdef LCD_2004
#define INITDISPLAY { xdisplay.begin(20,4);xdisplay.setBacklight(255) }
#define d_rect(a,b,c,d) 
#define d_frect(a,b,c,d) 
#define d_line(a,b,c,d) 

#define d_w() 200
#define d_h() 64
#define d_clear() xdisplay.home();xdisplay.clear();
#define d_show() 
#define d_setcolor(x) 
#define d_text(x,y,s) xdisplay.setCursor(x,y);xdisplay.print(s);
#define d_t_width(s) (s.length()*10)
#else


#ifdef LCD_UC1609
    //xdisplay.Initial();xdisplay.setContrast(190,false);xdisplay.setRotate(true)
    #ifdef LCD_UC1609DARK
        #define INITDISPLAY xdisplay.init();xdisplay.setContrast(190,false);xdisplay.setRotate(true); xdisplay.setFont(xfont);
        #else
        #define INITDISPLAY xdisplay.init();xdisplay.setContrast(140,false);xdisplay.setRotate(true);xdisplay.setFont(xfont);
    #endif
#else
    #ifdef LCD_NK1202
        #define INITDISPLAY xdisplay.init();xdisplay.setContrast(200,false);xdisplay.setFont(xfont);
    #else
        // other oled here
        #define INITDISPLAY xdisplay.init();xdisplay.setFont(xfont);
    #endif
#endif

#define d_rect(a,b,c,d) xdisplay.drawRect(a,b,c,d)
#define d_frect(a,b,c,d) xdisplay.fillRect(a,b,c,d)
#define d_line(a,b,c,d) xdisplay.drawLine(a,b,c,d)

#define d_w() 20
//xdisplay.getWidth()
#define d_h() 4
//xdisplay.getHeight()
#define d_clear() xdisplay.clear()
#define d_show() IR_end();xdisplay.display();IR_setup()
//only if IR pin same with LCD CS Pin
//#define d_show() display.display();IR_connect()
#define d_setcolor(x) xdisplay.setColor(x)
#define d_text(x,y,s) xdisplay.drawString((x)*LCD_X+XOFS,(y)*LCD_Y+YOFS,s)
#define d_t_width(s) xdisplay.getStringWidth(s)

#endif

extern void setup_oled(void);
extern int ir_oled_loop(int icommand);
#else
// if not using LCD

#define setup_oled() {}
#define ir_oled_loop(i) {}
#define d_clear()
#define d_show() 
#define d_setcolor(x) 
#define d_text(x,y,s)

#endif
