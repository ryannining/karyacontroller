//#pragma once
#ifndef iroled_H
#define iroled_H


#define USERGB12
#include "KaryaLCDWire.h"
extern KaryaLCD xdisplay;
extern int lIR_KEY,lcd_rst,lcd_sda,lcd_sda2,lcd_contrast,lcd_scl,lcd_addr,lcd_kind,XOFS,YOFS,LINES;
extern float LCD_X,LCD_Y;


extern void IR_setup();
extern void restartLCD();
#include "LiquidCrystal_I2C.h"
extern LiquidCrystal_I2C LCD;

#define d_w() 20
#define d_h() 4

extern int ir_oled_loop(int icommand);
extern int d_t_width(String s);
extern void  d_text(int x,int y,String s);
extern void d_setcolor(OLEDDISPLAY_COLOR x);
extern void d_show();
extern void d_clear();
extern void d_line(int a,int b,int c,int d);
extern void d_frect(int a,int b,int c,int d);
extern void d_rect(int a,int b,int c,int d);

#endif
