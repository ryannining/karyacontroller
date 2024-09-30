#pragma once
/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */
// Define which LCD types to include
#define INCLUDE_NK1661 0
#define INCLUDE_NK1202 0
#define INCLUDE_NK6100 0
#define INCLUDE_OLED1306 1
#define INCLUDE_ST7565 1
#define INCLUDE_ST7735 0


enum DISPLAY_KIND {
  KIND_NK1661   = 0,
  KIND_NK1202,
  KIND_NK6100,
  KIND_OLED1306,
  KIND_ST7565,
  KIND_ST7735,
  KIND_LCD2004,
};
enum OLEDDISPLAY_COLOR {
  BLACK = 0,
  WHITE = 1,
  INVERSE = 2
};
// Header Values
#define JUMPTABLE_BYTES 4

#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3
#define BACK_BLOCK 1

// Display commands
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2


#define LCDTYPE 1 // 1 = invert
#include <Arduino.h>
#include <Wire.h>
extern int motionloop();
#define uint unsigned int
#define uchar unsigned char


#define SDA this->_sda //serial data input
#define SDA2 this->_sda2 //serial data input
#define SCLK this->_scl //serial clock input
#define FCS this->_pcs //serial clock input
#define FSDA this->_psda //serial data input
#define FSDA2 this->_psda2 //serial data input
#define FSCLK this->_pscl //serial clock input

#define NOP __asm__ __volatile__("nop");

#define PIN_ON(pin) GPOS = (1 << pin)
#define PIN_OFF(pin) GPOC = (1 << pin)

#define SCLK_1 GPOS = FSCLK
#define SCLK_0 GPOC = FSCLK
#define SDAe(n)      \
    if (n) GPOS = FSDA; \
    else  GPOC = FSDA;  \
    SCLK_1;SCLK_0;

#define ISDATA  {GPOS = FSDA; SCLK_1;SCLK_0;}
#define ISCMD   {GPOC = FSDA; SCLK_1;SCLK_0;}
#define ISDATA2  {GPOS = FSDA2;}
#define ISCMD2   {GPOC = FSDA2;} 
//#define ISDATA2  {digitalWrite(SDA2,HIGH);}
//#define ISCMD2   {digitalWrite(SDA2,LOW);} 
#define CS_ON if (_cs!=_scl)GPOC=FCS;
#define CS_OFF {GPOS = FSDA;if (_cs!=_scl)GPOS=FCS;}
//#define CS_ON if (_cs!=_scl)digitalWrite(_cs,LOW);
//#define CS_OFF if (_cs!=_scl)digitalWrite(_cs,HIGH);
//#define CS_ON
//#define CS_OFF

   
const uchar Contrast_level = 140;


// Some register settings
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST7735_NOP 0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09

#define ST7735_SLPIN 0x10
#define ST7735_SLPOUT 0x11
#define ST7735_PTLON 0x12
#define ST7735_NORON 0x13

#define ST7735_INVOFF 0x20
#define ST7735_INVON 0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON 0x29
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_RAMRD 0x2E

#define ST7735_PTLAR 0x30
#define ST7735_TEOFF 0x34
#define ST7735_TEON 0x35
#define ST7735_MADCTL 0x36
#define ST7735_COLMOD 0x3A

#define ST7735_MADCTL_MY 0x80
#define ST7735_MADCTL_MX 0x40
#define ST7735_MADCTL_MV 0x20
#define ST7735_MADCTL_ML 0x10
#define ST7735_MADCTL_RGB 0x00

#define ST7735_RDID1 0xDA
#define ST7735_RDID2 0xDB
#define ST7735_RDID3 0xDC
#define ST7735_RDID4 0xDD


// clang-format off
static const uint8_t PROGMEM
                     //     255 = max (500 ms) delay

  Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
    15,                             // 15 commands in list:
    ST7735_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST7735_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF,  0,              // 13: Don't invert display, no args
    ST7735_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      0xC8,                         //     row/col addr, bottom-top refresh
    ST7735_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05 },                       //     16-bit color



  Rcmd3[] = {                       // 7735R init, part 3 (red or green tab)
    4,                              //  4 commands in list:
    ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
      0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
      0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST7735_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay

#define EPSONLCD

//********************************************************************
//
//					EPSON Controller Definitions
//
//********************************************************************
#define DISON       0xAF	// Display on
#define DISOFF      0xAE	// Display off
#define DISNOR      0xA6	// Normal display
#define DISINV      0xA7	// Inverse display
#define SLPIN       0x95	// Sleep in
#define SLPOUT      0x94	// Sleep out
#define COMSCN      0xBB	// Common scan direction
#define DISCTL      0xCA	// Display control
#define PASET       0x75	// Page address set
#define CASET       0x15	// Column address set
#define DATCTL      0xBC	// Data scan direction, etc.
#define RGBSET8     0xCE	// 256-color position set
#define RAMWR       0x5C	// Writing to memory
#define RAMRD       0x5D	// Reading from memory
#define PTLIN       0xA8	// Partial display in
#define PTLOUT      0xA9	// Partial display out
#define RMWIN       0xE0	// Read and modify write
#define RMWOUT      0xEE	// End
#define ASCSET      0xAA	// Area scroll set
#define SCSTART     0xAB	// Scroll start set
#define OSCON       0xD1	// Internal oscillation on
#define OSCOFF      0xD2	// Internal osciallation off
#define PWRCTR      0x20	// Power control
#define VOLCTR      0x81	// Electronic volume control
#define VOLUP       0xD6	// Increment electronic control by 1
#define VOLDOWN     0xD7	// Decrement electronic control by 1
#define TMPGRD      0x82	// Temperature gradient set
#define EPCTIN      0xCD	// Control EEPROM
#define EPCOUT      0xCC	// Cancel EEPROM control
#define EPMWR       0xFC	// Write into EEPROM
#define EPMRD       0xFD	// Read from EEPROM
#define EPSRRD1     0x7C	// Read register 1
#define EPSRRD2     0x7D	// Read register 2
#define NOP         0x25	// No op

//********************************************************************
//
//			PHILLIPS Controller Definitions
//
//********************************************************************
//LCD Commands
#define	NOPP		0x00	// No operation
#define	BSTRON		0x03	// Booster voltage on
#define SLEEPIN     0x10	// Sleep in
#define	SLEEPOUT	0x11	// Sleep out
#define	NORON		0x13	// Normal display mode on
#define	INVOFF		0x20	// Display inversion off
#define INVON      	0x21	// Display inversion on
#define	SETCON		0x25	// Set contrast
#define DISPOFF     0x28	// Display off
#define DISPON      0x29	// Display on
#define CASETP      0x2A	// Column address set
#define PASETP      0x2B	// Page address set
#define RAMWRP      0x2C	// Memory write
#define RGBSET	    0x2D	// Color set
#define	MADCTL		0x36	// Memory data access control
#define	COLMOD		0x3A	// Interface pixel format
#define DISCTR      0xB9	// Super frame inversion
#define	EC			0xC0	// Internal or external oscillator

//*******************************************************
//				12-Bit Color Definitions
//*******************************************************
/*
#define BLACK		0x000
#define NAVY 		0x008
#define BLUE		0x00F
#define TEAL 		0x088
#define EMERALD		0x0C5
#define	GREEN		0x0F0
#define CYAN		0x0FF
#define SLATE 		0x244
#define INDIGO  	0x408
#define TURQUOISE	0x4ED
#define OLIVE 		0x682
#define MAROON 		0x800
#define PURPLE 		0x808
#define GRAY 		0x888
#define SKYBLUE		0x8CE
#define BROWN		0xB22
#define CRIMSON 	0xD13
#define ORCHID 		0xD7D
#define RED		0xF00
#define MAGENTA		0xF0F
#define ORANGE 		0xF40
#define PINK		0xF6A
#define CORAL 		0xF75
#define SALMON 		0xF87
#define GOLD 		0xFD0
#define YELLOW		0xFF0
#define WHITE		0xFFF
*/

#define SPFD54124B_SEND_CMD 0
#define SPFD54124B_SEND_DATA 0x100

#define SPFD54124B_CMD_NOP 0x00
#define SPFD54124B_CMD_RST 0x01
#define SPFD54124B_CMD_SLPOUT 0x11
#define SPFD54124B_CMD_NORON 0x13

#define SPFD54124B_CMD_INVOFF 0x20
#define SPFD54124B_CMD_INVON 0x21
#define SPFD54124B_CMD_DISPON 0x29
#define SPFD54124B_CMD_CASET 0x2A
#define SPFD54124B_CMD_RASET 0x2B
#define SPFD54124B_CMD_RAMWR 0x2C
#define SPFD54124B_CMD_RGBSET 0x2D

#define SPFD54124B_CMD_MADCTR 0x36
#define SPFD54124B_CMD_VSCSAD 0x37
#define SPFD54124B_CMD_COLMOD 0x3A

#define SPFD54124B_CMD_COLMOD_MCU12bit 3 // MCU interface 12bit
#define SPFD54124B_CMD_COLMOD_MCU16bit 5 // MCU interface 16bit
#define SPFD54124B_CMD_COLMOD_MCU18bit 6 // MCU interface 18bit
#define SPFD54124B_CMD_COLMOD_RGB16bit 50 // RGB interface 16bit
#define SPFD54124B_CMD_COLMOD_RGB18bit 60 // RGB interface 18bit

#define SPFD54124B_CMD_MADCTR_MY (1 << 7) // Row Address Order
#define SPFD54124B_CMD_MADCTR_MX (1 << 6) // Column Address Order
#define SPFD54124B_CMD_MADCTR_MV (1 << 5) // Row/Column Exchange
#define SPFD54124B_CMD_MADCTR_ML (1 << 4) // Vertical Refresh Order
#define SPFD54124B_CMD_MADCTR_RGB (1 << 3) // RGB-BGR ORDER
#define SPFD54124B_CMD_SETPWCTR1               (0xB1)
#define SPFD54124B_CMD_SETPWCTR2               (0xB2)
#define SPFD54124B_CMD_SETPWCTR3               (0xB3)
#define SPFD54124B_CMD_SETPWCTR4               (0xB4)
#define SPFD54124B_CMD_SETPWCTR5               (0xB5)

#define ALIGN_LEFT 0
#define ALIGN_RIGHT -1
#define ALIGN_CENTER -2

#define SET 1
#define CLR 0
#define XOR 2


//#define FLIPMODE

//#undef USERGB12

#define RGB12(r,g,b) ((r>>1) | ((g>>2)<<4) | ((b>>1)<<8))
#define RGB16a(r,g,b) ((r) | (g<<5) | (b<<11))

#ifdef USERGB12
#define RGB16 RGB12
#else
#define RGB16 RGB16a
#endif


// HX1230 Commands
#define HX1230_POWER_ON 0x2F // internal power supply on
#define HX1230_POWER_OFF 0x28 // internal power supply off
#define HX1230_CONTRAST 0x80 // 0x80 + (0..31)
#define HX1230_SEG_NORMAL 0xA0 // SEG remap normal
#define HX1230_SEG_REMAP 0xA1 // SEG remap reverse (flip horizontal)
#define HX1230_DISPLAY_NORMAL 0xA4 // display ram contents
#define HX1230_DISPLAY_TEST 0xA5 // all pixels on
#define HX1230_INVERT_OFF 0xA6 // not inverted
#define HX1230_INVERT_ON 0xA7 // inverted
#define HX1230_DISPLAY_ON 0XAF // display on
#define HX1230_DISPLAY_OFF 0XAE // display off
#define HX1230_SCAN_START_LINE 0x40 // scrolling 0x40 + (0..63)
#define HX1230_COM_NORMAL 0xC0 // COM remap normal
#define HX1230_COM_REMAP 0xC8 // COM remap reverse (flip vertical)
#define HX1230_SW_RESET 0xE2 // connect RST pin to GND and rely on software reset
#define HX1230_COM_VOP 0xE1 // connect RST pin to GND and rely on software reset

#define HX1230_68_LINE 0xD0 // no operation
#define HX1230_NOP 0xE3 // no operation
#define HX1230_COL_ADDR_H 0x10 // x pos (0..95) 4 MSB
#define HX1230_COL_ADDR_L 0x00 // x pos (0..95) 4 LSB
#define HX1230_PAGE_ADDR 0xB0 // y pos, 8.5 rows (0..8)




/* ===================
ST565
*/

#define CMD_DISPLAY_OFF   0xAE
#define CMD_DISPLAY_ON    0xAF

#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE  0xB0

#define CMD_SET_COLUMN_UPPER  0x10
#define CMD_SET_COLUMN_LOWER  0x00

#define CMD_SET_ADC_NORMAL  0xA0
#define CMD_SET_ADC_REVERSE 0xA1

#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7

#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON  0xA5
#define CMD_SET_BIAS_9 0xA2 
#define CMD_SET_BIAS_7 0xA3

#define CMD_RMW  0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET  0xE2
#define CMD_SET_COM_NORMAL  0xC0
#define CMD_SET_COM_REVERSE  0xC8
#define CMD_SET_POWER_CONTROL  0x28
#define CMD_SET_RESISTOR_RATIO  0x20
#define CMD_SET_VOLUME_FIRST  0x81
#define  CMD_SET_VOLUME_SECOND  0
#define CMD_SET_STATIC_OFF  0xAC
#define  CMD_SET_STATIC_ON  0xAD
#define CMD_SET_STATIC_REG  0x0
#define CMD_SET_BOOSTER_FIRST  0xF8
#define CMD_SET_BOOSTER_234  0
#define  CMD_SET_BOOSTER_5  1
#define  CMD_SET_BOOSTER_6  3
#define CMD_NOP  0xE3
#define CMD_TEST  0xF0
#define ST7565_STARTBYTES 1
// 
const uint8_t initData[] PROGMEM = {
    
    
    HX1230_DISPLAY_NORMAL,
    HX1230_POWER_ON,
    HX1230_DISPLAY_ON,
    

};
//--------------------------------------

class KaryaLCD {
private:
    uint8_t _sda,_psda,_sda2,_psda2;
    uint8_t _scl,_pscl,_pcs,bctr;
    uint8_t _cs,_kind,_address;

    bool justreset;
    bool _doI2cAutoInit = false;    
uint8_t            *buffer;

    uint8_t            *buffer_back;


  protected:


    DISPLAY_KIND kind;

    uint16_t  displayWidth;
    uint16_t  displayHeight;
    uint16_t  displayBufferSize;

    // Set the correct height, width and buffer for the geometry

    OLEDDISPLAY_COLOR            color;

    const uint8_t	 *fontData;

public:
    uint16_t hpos,col,hcol,bg,hbg;
    uint8_t _rs;
    KaryaLCD(uint8_t lcdkind,uint8_t _address,uint8_t sda, uint8_t sda2,uint8_t scl, uint8_t cs,uint8_t rs,uint8_t tpos=21)
    {
        this->setparam(lcdkind,_address,sda,sda2,scl,cs,rs,tpos);

    }
    void setparam(uint8_t lcdkind,uint8_t _address,uint8_t sda, uint8_t sda2,uint8_t scl, uint8_t cs,uint8_t rs,uint8_t tpos=21)
    {
        if (rs==255)rs=scl;
        if (cs==255)cs=scl;
        this->_kind=lcdkind;
        this->_sda = sda;
        this->_sda2 = sda2;
        this->_scl = scl;
        this->_cs = cs;
        this->_rs = rs;
        this->_psda = 1<<sda;
        this->_psda2= 1<<sda2;
        this->_pscl = 1<<scl;
        this->_pcs = 1<<cs;
        this->_address = _address;
        #ifdef INCLUDE_ST7735
        if (lcdkind==KIND_ST7735){
          col=RGB16a(31,63,31);
          bg=RGB16a(0,0,0);
          hcol=RGB16a(5,63,5);
          hbg=RGB16a(0,0,28);
        } else 
        #else
        {
          col=RGB16(31,63,31);
          bg=RGB16(0,0,0);
          hcol=RGB16(5,63,5);
          hbg=RGB16(0,0,28);
        }
        #endif
        hpos=tpos;
    }
    bool connect()
    {
        #if INCLUDE_OLED1306
        if (kind==KIND_OLED1306){
            Wire.begin(this->_sda, this->_scl);
            Wire.setClock(700000);
        }
        #endif
        return true;
    }
    #if INCLUDE_ST7735
    void displayInits(const uint8_t *addr) {

      uint8_t numCommands, cmd, numArgs;
      uint16_t ms;

      numCommands = pgm_read_byte(addr++); // Number of commands to follow
      while (numCommands--) {              // For each command...
        cmd = pgm_read_byte(addr++);       // Read command
        numArgs = pgm_read_byte(addr++);   // Number of args to follow
        ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
        writeCommand(cmd);
        for (int i=numArgs;i;i--){
          writeData(pgm_read_byte(addr++));
        }

        if (ms) {
          ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
          if (ms == 255)
            ms = 500; // If 255, delay for 500 ms
          delay(ms);
        }
      }
    }    
    #endif
    bool allocateBuffer();
    void end();
    bool init()
    {
        if (_sda==255)return false;
        
        switch (_kind){ 
            #if INCLUDE_NK1661
            case KIND_NK1661:    displayWidth=162;displayHeight=132;break;
            #endif
            #if INCLUDE_NK1202
            case KIND_NK1202:    displayWidth=96;displayHeight=68;break;
            #endif
            #if INCLUDE_NK6100
            case KIND_NK6100:    displayWidth=128;displayHeight=128;break;
            #endif
            #if INCLUDE_OLED1306
            case KIND_OLED1306:  displayWidth=128;displayHeight=64;break;
            #endif
            #if INCLUDE_ST7565
            case KIND_ST7565:    displayWidth=128;displayHeight=64;break;
            #endif
            #if INCLUDE_ST7735
            case KIND_ST7735:    displayWidth=160;displayHeight=128;break;
            #endif
        }   
        displayBufferSize=displayWidth*displayHeight/8;
        
        if (!allocateBuffer()) {
            return false;
        }
        Reset();
        //clear();
        memset(buffer_back, 0, displayBufferSize);
        memset(buffer, 0, displayBufferSize);
        fillScreen(0);
        return true;
    }    
    int display(void){
     
        if (_sda==255)return 0;
        switch (_kind){
            #if INCLUDE_NK1661
            case KIND_NK1661:   return displayNK1661();break;
            #endif
            #if INCLUDE_ST7735
            case KIND_ST7735:   return displayST7735();break;
            #endif
            #if INCLUDE_NK1202
            case KIND_NK1202:   return displayNK1202();break;
            #endif
            #if INCLUDE_NK6100
            case KIND_NK6100:   return displayNK6100();break;
            #endif
            #if INCLUDE_OLED1306
            case KIND_OLED1306: return displayOLED1306();break;
            #endif
            #if INCLUDE_ST7565
            case KIND_ST7565: return displayST7565();break;
            #endif
        }
   
      return 0;
    }
    

    #if INCLUDE_NK6100
    void setWindow6100(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
    {
        uint8_t t0, t1;
        if (_sda==255)return;
        #ifdef EPSONLCD
        writeCommand(PASET);  // page start/end ram
        writeData(x0);
        writeData(132);

        writeCommand(CASET);  // column start/end ram
        writeData(y0);
        writeData(130);

        writeCommand(RAMWR);
        #else
        writeCommand(PASETP); // page start/end ram
        writeData(x0);
        writeData(x1);

        writeCommand(CASETP); // column start/end ram
        writeData(y0);
        writeData(y1);

        writeCommand(RAMWRP); // write
        #endif
    }
    #endif
    #if INCLUDE_NK1661
    void setWindow1661(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
    {
        uint8_t t0, t1;
        if (_sda==255)return;
        writeCommand(SPFD54124B_CMD_CASET); // Column addr set
        //writeData(x0 >>8); // X start
        writeData16(x0); // X start
        //writeData(x1 >>8); // X end
        writeData16(x1); // X end

        writeCommand(SPFD54124B_CMD_RASET); // Page addr set
        //writeData(y0>>8);
        writeData16(y0);
        //writeData(y1>>8);
        writeData16(y1);

        writeCommand(SPFD54124B_CMD_RAMWR);
    }    
    #endif
    #if INCLUDE_ST7735
    void setWindowst7735(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
      writeCommand(ST7735_CASET); // Column addr set
      writeData16(x0);
      writeData16(x1);

      writeCommand(ST7735_RASET); // Row addr set
      writeData16(y0);
      writeData16(y1);
      
      writeCommand(ST7735_RAMWR); // write to RAM
  }
    #endif
    #if INCLUDE_NK1202  
    void setWindow1202(uint16_t x0, uint16_t y0)
    {
        uint8_t t0, t1;
        if (_sda==255)return;
        writeCommand(HX1230_PAGE_ADDR | y0);
        writeCommand(HX1230_COL_ADDR_H | (x0 >> 4));
        writeCommand(HX1230_COL_ADDR_L | (x0 & 0xf));

    }    
    #endif
    #if INCLUDE_OLED1306
    void setWindow1306(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
        sendCommand(COLUMNADDR);
        sendCommand(x0);
        sendCommand(x1);

        sendCommand(PAGEADDR);
        sendCommand(y0);
        sendCommand(y1);
    }
    #endif
    #if INCLUDE_ST7565
    void setWindowst7565(uint16_t x0,uint16_t y0)
    {
        uint8_t t0, t1;
        //const uint8_t pagemap[] = { 3, 2, 1, 0, 7, 6, 5, 4 };
        const uint8_t pagemap[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        if (_sda==255)return;
        /*
        writeCommand(CMD_SET_PAGE | pagemap[y0]);
        writeCommand(CMD_SET_COLUMN_LOWER | ((x0+ST7565_STARTBYTES) & 0xf));
        writeCommand(CMD_SET_COLUMN_UPPER | (((x0+ST7565_STARTBYTES) >> 4) & 0x0F));
        writeCommand(CMD_RMW);
        */
        writeCommand(CMD_SET_PAGE | y0);
        writeCommand(CMD_SET_COLUMN_LOWER |  (x0 & 0xf));
        writeCommand(CMD_SET_COLUMN_UPPER | ((x0 >> 4) & 0x0F));
        writeCommand(CMD_RMW);
    }  
    #endif

    void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
      switch (_kind){
        #if INCLUDE_NK1661
            case KIND_NK1661:   setWindow1661(x0,y0,x1,y1);break;
        #endif
        #if INCLUDE_NK1202
            case KIND_NK1202:   setWindow1202(x0,y0);break;
        #endif
        #if INCLUDE_NK6100
            case KIND_NK6100:   setWindow6100(x0,y0,x1,y1);break;
        #endif
        #if INCLUDE_OLED1306
            case KIND_OLED1306: setWindow1306(x0,y0,x1,y1);break;
        #endif
        #if INCLUDE_ST7565
            case KIND_ST7565:   setWindowst7565(x0,y0);break;
        #endif
        #if INCLUDE_ST7735
            case KIND_ST7735:   setWindowst7735(x0,y0,x1,y1);break;
        #endif
      }
    }
    #if INCLUDE_NK1202      
    int displayNK1202(void)
    {


        long m = micros();

        pinMode(_sda, OUTPUT);
        CS_ON
        uint8_t x, y,npx;
        extern int motionloop();
        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < 9; y++) {
            uint8_t minBoundX = UINT8_MAX;
            npx=0;
            for (x = 0; x <= displayWidth; x++) {
                uint16_t pos = x + y * displayWidth;
                if ((x<displayWidth) && (justreset || (buffer[pos] != buffer_back[pos]))) {
                    if (minBoundX==UINT8_MAX)minBoundX =x;
                    buffer_back[pos] = buffer[pos];
                    npx++;
                } else {
                    // draw a chunk of display
                    if ((minBoundX!=UINT8_MAX) && (npx>(x<displayWidth?10:0))) { // minimum 10pixel at once

                        
                        uint8_t maxBoundX=_min(x+1,displayWidth);
                        setWindow1202(minBoundX,y);
                        uint8_t* pdat = &buffer[minBoundX + y * displayWidth];
                        for (uint8_t dx = minBoundX; dx < maxBoundX; dx++) {

                                uint8_t dat = *pdat;
                                //if (y==8)dat=0xff;
                                pdat++;
                                ISDATA;
                                SDAe(dat&(1<<7));
                                SDAe(dat&(1<<6));
                                SDAe(dat&(1<<5));
                                SDAe(dat&(1<<4));
                                SDAe(dat&(1<<3));
                                SDAe(dat&(1<<2));
                                SDAe(dat&(1<<1));
                                SDAe(dat&(1<<0));
                        }

                        // reset counter find other chunk
                        minBoundX = UINT8_MAX;
                        npx=0;
                        

                    }

                }
                
            }
            yield();
            motionloop();
        }
    CS_OFF
    justreset=false;
    return micros()-m;

    }
    #endif
    #if INCLUDE_ST7565
    int displayST7565(void)
    {


        long m = micros();

        pinMode(_sda, OUTPUT);
        pinMode(_sda2, OUTPUT);
        CS_ON
        uint8_t x, y,npx;
        extern int motionloop();
        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < 8; y++) {
            uint8_t minBoundX = UINT8_MAX;
            npx=0;
            for (x = 0; x <= displayWidth; x++) {
                uint16_t pos = x + y * displayWidth;
                if ((x<displayWidth) && (justreset || (buffer[pos] != buffer_back[pos]))) {
                    if (minBoundX==UINT8_MAX)minBoundX =x;
                    buffer_back[pos] = buffer[pos];
                    npx++;
                } else {
                    // draw a chunk of display
                    if ((minBoundX!=UINT8_MAX) && (npx>(x<displayWidth?10:0))) { // minimum 10pixel at once

                        
                        uint8_t maxBoundX=_min(x+1,displayWidth);
                        //setWindowst7565(minBoundX,y);
                        writeCommand(CMD_SET_PAGE | y);
                        writeCommand(CMD_SET_COLUMN_LOWER |  (minBoundX & 0xf));
                        writeCommand(CMD_SET_COLUMN_UPPER | ((minBoundX >> 4) & 0x0F));
                        writeCommand(CMD_RMW);
                        uint8_t* pdat = &buffer[minBoundX + y * displayWidth];
                        for (uint8_t dx = minBoundX; dx < maxBoundX; dx++) {

                                uint8_t dat = *pdat;
                                //if (y==8)dat=0xff;
                                pdat++;
                                ISDATA2;
                                SDAe(dat&(1<<7));
                                SDAe(dat&(1<<6));
                                SDAe(dat&(1<<5));
                                SDAe(dat&(1<<4));
                                SDAe(dat&(1<<3));
                                SDAe(dat&(1<<2));
                                SDAe(dat&(1<<1));
                                SDAe(dat&(1<<0));
                        }

                        // reset counter find other chunk
                        minBoundX = UINT8_MAX;
                        npx=0;
                        

                    }

                }
                
            }
            yield();
            motionloop();
        }
    CS_OFF
    justreset=false;
    return micros()-m;

    }
    #endif
    #if INCLUDE_OLED1306
    int displayOLED1306(void) {
      initI2cIfNeccesary();
      const int x_offset = (128 - displayWidth) / 2;

        uint8_t minBoundY = UINT8_MAX;
        uint8_t maxBoundY = 0;

        uint8_t minBoundX = UINT8_MAX;
        uint8_t maxBoundX = 0;
        uint8_t x, y;

        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < (displayHeight / 8); y++) {
          for (x = 0; x < displayWidth; x++) {
           uint16_t pos = x + y * displayWidth;
           if (buffer[pos] != buffer_back[pos])
           {
             minBoundY = _min(minBoundY, y);
             maxBoundY = _max(maxBoundY, y);
             minBoundX = _min(minBoundX, x);
             maxBoundX = _max(maxBoundX, x);
           }
           buffer_back[pos] = buffer[pos];
         }
         yield();
        }

        // If the minBoundY wasn't updated
        // we can savely assume that buffer_back[pos] == buffer[pos]
        // holdes true for all values of pos

        if (minBoundY == UINT8_MAX) return 0;
        setWindow1306(x_offset + minBoundX,minBoundY,x_offset + maxBoundX,maxBoundY);

        byte k = 0;
        for (y = minBoundY; y <= maxBoundY; y++) {
          for (x = minBoundX; x <= maxBoundX; x++) {
            if (k == 0) {
              Wire.beginTransmission(_address);
              Wire.write(0x40);
            }

            Wire.write(buffer[x + y * displayWidth]);
            k++;
            if (k == 16)  {
              Wire.endTransmission();
              k = 0;
            }
          }
          yield();
        }

        if (k != 0) {
          Wire.endTransmission();
        }
        return 1;
      
    }
    #endif
    
    void setI2cAutoInit(bool doI2cAutoInit) {
      _doI2cAutoInit = doI2cAutoInit;
    }
    #if INCLUDE_NK1661  
    int displayNK1661(void)
    {

        long m = micros();
        pinMode(_scl, OUTPUT);
        pinMode(_cs, OUTPUT);
        pinMode(_rs, OUTPUT);
        pinMode(_sda, OUTPUT);
        
        

        

        uint8_t x, y,npx;
        extern int motionloop();
        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < (displayHeight / 8); y++) {
            uint8_t minBoundX = UINT8_MAX;
            npx=0;
            for (x = 0; x <= displayWidth; x++) {
                uint16_t pos = x + y * displayWidth;
                if ((x<displayWidth) && (justreset || (buffer[pos] != buffer_back[pos]))) {
                    npx++;
                    buffer_back[pos] = buffer[pos];
                    if (minBoundX==UINT8_MAX)minBoundX = x;
                } else {
                    // draw a chunk of display
                    if ((minBoundX!=UINT8_MAX) && (npx>(x<displayWidth?10:0))) { // minimum 10pixel at once

                        uint8_t maxBoundX=_min(x+1,displayWidth);
                        uint8_t* pdat = &buffer[minBoundX + y * displayWidth];
                        CS_ON
#if FLIPMODE
                        setWindow((y) * 8, minBoundX+1,(y+1) * 8 -1, maxBoundX);
#else
                        //setWindow(130 - (y + 1) * 8, minBoundX+1,130 - y * 8 -1, maxBoundX);
                        setWindow(132 - (y+1) * 8, minBoundX,131 - y * 8 , maxBoundX-1);
#endif

                        int nn=0;
                        for (uint8_t dx = minBoundX; dx < maxBoundX; dx++) {
                            // send 8pixel /byte
                            uint8_t dat = *pdat;

                            pdat++;
                            int hy=(y+1)*8;
                            int16_t ccol=col;
                            uint16_t cbg=bg;
                            CS_ON
                            if (hy--<=hpos){ccol=hcol;cbg=hbg;}
                            for (uint8_t p = 0x80; p; p>>=1) {
                                
                                #ifdef RGB12
                                    writeData12(
                                #else
                                    writeData16(
                                #endif
                                    (dat & p ? ccol:cbg));
                                
                                nn++;
                                //if ((nn>300) && (bctr==7)){
                                    //CS_OFF
                                    //nn=0;
                                    //motionloop();
                                    //CS_ON
                                //}
                            }

                        }
                        #ifdef RGB12
                        for (bctr++;bctr<8;bctr++){
                            SCLK_1;
                            SCLK_0;
                        }
                        #endif
                        // reset counter find other chunk
                        minBoundX = UINT8_MAX;
                        npx=0;
                        //yield();
                        CS_OFF
                        motionloop();         
                                      
                    }
                    
                }

            }
        }

    justreset=false;
    //SDAe(1);
        return 0;//micros()-m;

    }
    #endif
    #if INCLUDE_ST7735
    int displayST7735(void)
    {

        long m = micros();
        pinMode(_scl, OUTPUT);
        pinMode(_cs, OUTPUT);
        pinMode(_rs, OUTPUT);
        pinMode(_sda, OUTPUT);
        pinMode(_sda2, OUTPUT);
        
        

        

        uint8_t x, y,npx;
        extern int motionloop();
        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < (displayHeight / 8); y++) {
            uint8_t minBoundX = UINT8_MAX;
            npx=0;
            for (x = 0; x <= displayWidth; x++) {
                uint16_t pos = x + y * displayWidth;
                if ((x<displayWidth) && (justreset || (buffer[pos] != buffer_back[pos]))) {
                    npx++;
                    buffer_back[pos] = buffer[pos];
                    if (minBoundX==UINT8_MAX)minBoundX = x;
                } else {
                    // draw a chunk of display
                    if ((minBoundX!=UINT8_MAX) && (npx>(x<displayWidth?10:0))) { // minimum 10pixel at once

                        uint8_t maxBoundX=_min(x+1,displayWidth);
                        uint8_t* pdat = &buffer[minBoundX + y * displayWidth];
                        CS_ON
#if FLIPMODE
                        setWindowst7735((y) * 8, minBoundX+1,(y+1) * 8 -1, maxBoundX);
#else
                        //setWindow(130 - (y + 1) * 8, minBoundX+1,130 - y * 8 -1, maxBoundX);
                        setWindowst7735(128 - (y+1) * 8, minBoundX,127 - y * 8 , maxBoundX-1);
#endif

                        int nn=0;
                        for (uint8_t dx = minBoundX; dx < maxBoundX; dx++) {
                            // send 8pixel /byte
                            uint8_t dat = *pdat;

                            pdat++;
                            int hy=(y+1)*8;
                            int16_t ccol=col;
                            uint16_t cbg=bg;
                            CS_ON
                            if (hy--<=hpos){ccol=hcol;cbg=hbg;}
                            for (uint8_t p = 0x80; p; p>>=1) {
                                
                                    writeData16((dat & p ? ccol:cbg));
                                
                                nn++;

                            }

                        }
                        // reset counter find other chunk
                        minBoundX = UINT8_MAX;
                        npx=0;
                        //yield();
                        CS_OFF
                        motionloop();         
                                      
                    }
                    
                }

            }
        }

    justreset=false;
    //SDAe(1);
        return 0;//micros()-m;

    }
    #endif
    #if INCLUDE_NK6100      
    int displayNK6100(void)
    {

        long m = micros();
        pinMode(_scl, OUTPUT);
        pinMode(_cs, OUTPUT);
        pinMode(_rs, OUTPUT);
        pinMode(_sda, OUTPUT);
        
        

        

        uint8_t x, y,npx;
        extern int motionloop();
        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < (displayHeight / 8); y++) {
            uint8_t minBoundX = UINT8_MAX;
            npx=0;
            for (x = 0; x <= displayWidth; x++) {
                uint16_t pos = x + y * displayWidth;
                if ((x<displayWidth) && (justreset || (buffer[pos] != buffer_back[pos]))) {
                    npx++;
                    buffer_back[pos] = buffer[pos];
                    if (minBoundX==UINT8_MAX)minBoundX = x;
                } else {
                    // draw a chunk of display
                    if ((minBoundX!=UINT8_MAX) && (npx>(x<displayWidth?10:0))) { // minimum 10pixel at once

                        uint8_t maxBoundX=_min(x+1,displayWidth);
                        uint8_t* pdat = &buffer[minBoundX + y * displayWidth];
                        CS_ON
#if FLIPMODE
                        setWindow6100((y) * 8, minBoundX+1,(y+1) * 8 -1, maxBoundX);
#else
                        setWindow6100(132 - (y+1) * 8, minBoundX,131 - y * 8 , maxBoundX-1);
#endif

                        int nn=0;
                        for (uint8_t dx = minBoundX; dx < maxBoundX; dx++) {
                            // send 8pixel /byte
                            uint8_t dat = *pdat;

                            pdat++;
                            int hy=(y+1)*8;
                            int16_t ccol=col;
                            uint16_t cbg=bg;
                            CS_ON
                            if (hy--<=hpos){ccol=hcol;cbg=hbg;}
                            for (uint8_t p = 0x80; p; p>>=1) {
                                    #ifdef EPSONLCD
                                    writeData16((dat & p ? ccol:cbg));
                                    writeData(0);
                                    #else
                                    writeData16((dat & p ? ccol:cbg));
                                    #endif
                                nn++;
                            }

                        }
                        #ifdef EPSONLCD
                        writeCommand(NOP);
                        #else
                        writeCommand(NOPP);
                        #endif
                        // reset counter find other chunk
                        minBoundX = UINT8_MAX;
                        npx=0;
                        //yield();
                        CS_OFF
                        motionloop();         
                                      
                    }
                    
                }

            }
        }

    justreset=false;
    //SDAe(1);
        return 0;//micros()-m;

    }    
    #endif  
    // clear everything
    void setLCDColor(uint16_t c1,uint16_t c2,uint16_t c3=0,uint16_t c4=0){
        col=c1;
        hcol=c2;
        bg=c3;
        hbg=c4;
        Reset();
    }
    void fillScreen(uint16_t c)
    {
        switch (_kind){
          case KIND_LCD2004:
          case KIND_OLED1306:return;
        }
        if (_sda==255)return;
        uint8_t x, y, hi = c >> 8, lo = c;

        CS_ON
        setWindow(0, 0, displayWidth-1, displayHeight-1);
        for (y = displayHeight/8; y > 0; y--) {
            for (x = displayWidth; x > 0; x--) {
                switch (_kind){
                  KIND_NK1202:writeData(0);break;
                  KIND_NK6100:writeData(0);
                  KIND_ST7565:writeData(0);
                  KIND_NK1661:writeData(lo);writeData(hi);break;
                }
                //lo++;
            }
            //hi++;
        }
        CS_OFF
    }

    int getBufferOffset(void)
    {
        return 0;
    }
    
    void writeany(uint16_t dat)
    {
        if (_sda==255)return;
        for (uint16_t i = 0x100; i ; i>>=1) {
            SDAe((dat & i));
        }
        return;
    }
    void writeCommand(unsigned char dat)
    {
        if (_sda==255)return;
        if (_sda2==255)ISCMD else ISCMD2 
        for (uint8_t i = 0x80; i ; i>>=1) {
            SDAe((dat & i));
        }
        bctr=7;
        return;
    }
    void writeData(uint16_t dat)
    {
        if (_sda2==255)ISDATA else ISDATA2 
        for (uint8_t i = 1<<7; i; i>>=1) {
            SDAe((dat & i));
        }

        return;
    }
    void writeData12(uint16_t dat)
    {        
        if (_sda==255)return;
        for (uint16_t i = 1<<11; i; i>>=1) {
            if (++bctr==8){
                ISDATA;
                bctr=0;
            }
            SDAe((dat & i));
        }
        return;
    }
    void writeData16(uint16_t dat)
    {        
        for (uint16_t i = 1 << 15; i; i>>=1) {
            if (++bctr==8){
                if (_sda2==255)ISDATA else ISDATA2
                bctr=0;
            }
            SDAe((dat & i));
        }
        return;
    }
    void setContrast(uint8_t mod, bool rev = false)
    {
    }
    #if INCLUDE_OLED1306
    void resetOled1306(void) {

        sendCommand(DISPLAYOFF);
        sendCommand(SETDISPLAYCLOCKDIV);
        sendCommand(0xF0); // Increase speed of the display max ~96Hz
        sendCommand(SETMULTIPLEX);
        sendCommand(displayHeight - 1);
        sendCommand(SETDISPLAYOFFSET);
        sendCommand(0x00);
        sendCommand(SETSTARTLINE);
        sendCommand(CHARGEPUMP);
        sendCommand(0x14);
        sendCommand(MEMORYMODE);
        sendCommand(0x00);
        sendCommand(SEGREMAP);
        sendCommand(COMSCANINC);
        sendCommand(SETCOMPINS);

        if (displayWidth == 128 && displayHeight == 64) {
            sendCommand(0x12);
        } else if (displayWidth == 128 && displayHeight == 32) {
            sendCommand(0x02);
        }

        sendCommand(SETCONTRAST);

        if (displayWidth == 128 && displayHeight == 64) {
            sendCommand(0xCF);
        } else if (displayWidth == 128 && displayHeight == 32) {
            sendCommand(0x8F);
        }

        sendCommand(SETPRECHARGE);
        sendCommand(0xF1);
        sendCommand(SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
        sendCommand(0x40);	        //0x40 default, to lower the contrast, put 0
        sendCommand(DISPLAYALLON_RESUME);
        sendCommand(NORMALDISPLAY);
        sendCommand(0x2e);            // stop scroll
        sendCommand(DISPLAYON);
    }
    #endif
    void Reset(void){        
        switch (_kind){ 
            #if INCLUDE_NK1661
            case KIND_NK1661:   ResetNK1661();break;
            #endif
            #if INCLUDE_NK6100
            case KIND_NK6100:   ResetNK6100();break;
            #endif
            #if INCLUDE_NK1202
            case KIND_NK1202:   ResetNK1202();break;
            #endif
            #if INCLUDE_ST7565
            case KIND_ST7565:   ResetST7565();break;
            #endif
            #if INCLUDE_ST7735
            case KIND_ST7735:   ResetST7735();break;
            #endif
            #if INCLUDE_OLED1306
            case KIND_OLED1306:  resetOled1306();break;
            #endif
        }
    }

    void ResetSPI(void){
        pinMode(_sda, OUTPUT);
        pinMode(_scl, OUTPUT);
        pinMode(_cs, OUTPUT);
        pinMode(_rs, OUTPUT);
        digitalWrite(_sda, LOW);
        digitalWrite(_scl, LOW);
        digitalWrite(_cs, LOW);
        digitalWrite(_rs, LOW);
        int dl=0;    
        if (_rs ==255) _rs=_scl;    
        if (_rs != _scl){
            extern int mstep_pin[4];
            bool L=(_rs==_sda);
            int D=100;
            
            digitalWrite(_rs,LOW); delay(D+(L?1000:0));
            digitalWrite(_rs,HIGH); delay(D);
        } else {
            //digitalWrite(_rs,HIGH); delay(250);
            //digitalWrite(_rs,LOW); delay(250);
        } 
        if (_cs == _scl){
            dl=50 + (_rs == _scl?100:0);
            digitalWrite(_cs,HIGH);
            delay(dl);
            digitalWrite(_cs,LOW);
            delay(dl*2);
        } else {
            CS_OFF
            delay(1);
            CS_ON
        }
        justreset=true; 
    }
    #if INCLUDE_ST7735
    void ResetST7735(void){
      ResetSPI();
      displayInits(Rcmd1);
      displayInits(Rcmd3);
      writeCommand(ST7735_MADCTL);
      writeData(0xC0);
      writeCommand(ST7735_MADCTL);
      writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
      writeCommand(ST7735_DISPON);
    }
    #endif
    #if INCLUDE_NK6100
    void ResetNK6100(void){
      ResetSPI();

      
      #ifdef EPSONLCD
      writeCommand(DISCTL);	// Display control (0xCA)
      writeData(0x0C);		// 12 = 1100 - CL dividing ratio [don't divide] switching period 8H (default)
      writeData(0x20);		// nlines/4 - 1 = 132/4 - 1 = 32 duty
      writeData(0x00);		// No inversely highlighted lines
      
      writeCommand(COMSCN);	// common scanning direction (0xBB)
      writeData(0x01);		// 1->68, 132<-69 scan direction
      
      writeCommand(OSCON);	// internal oscialltor ON (0xD1)
      writeCommand(SLPOUT);	// sleep out (0x94)
      
      writeCommand(PWRCTR);	// power ctrl (0x20)
      writeData(0x0F);		// everything on, no external reference resistors
      
      writeCommand(DISINV);	// invert display mode (0xA7)
      
      writeCommand(DATCTL);	// data control (0xBC)
      writeData(0x03);		// Inverse page address, reverse rotation column address, column scan-direction	!!! try 0x01
      writeData(0x00);		// normal RGB arrangement
      writeData(0x02);		// 16-bit Grayscale Type A (12-bit color)
      
      writeCommand(VOLCTR);	// electronic volume, this is the contrast/brightness (0x81)
      writeData(32);		// volume (contrast) setting - fine tuning, original (0-63)
      writeData(3);			// internal resistor ratio - coarse adjustment (0-7)
      
      writeCommand(NOP);	// nop (0x25)

      delay(100);

      writeCommand(DISON);	// display on (0xAF)
      #else
        writeCommand(SLEEPOUT);	// Sleep Out (0x11)
      writeCommand(BSTRON);   	// Booster voltage on (0x03)
      writeCommand(DISPON);		// Display on (0x29)
      
      //writeCommand(INVON);		// Inversion on (0x20)
      
      // 12-bit color pixel format:
      writeCommand(COLMOD);		// Color interface format (0x3A)
      writeData(0x03);			// 0b011 is 12-bit/pixel mode
      
      writeCommand(MADCTL);		// Memory Access Control(PHILLIPS)

      writeData(0x00);
      
      writeCommand(SETCON);		// Set Contrast(PHILLIPS)
      writeData(0x30);
      
      writeCommand(NOPP);		// nop(PHILLIPS)
      #endif
      CS_OFF
    }
    #endif
    #if INCLUDE_NK1202
    void ResetNK1202(void){
        
        ResetSPI();
        uint8_t size = 5;

        for (int i = 0; i < sizeof(initData); i++)
            writeCommand(pgm_read_byte(initData + i));

        CS_OFF
    }
    #endif
    #if INCLUDE_NK1661
    void ResetNK1661(void){
        if (_sda==255)return;
        #define DELAY 0x80
        const uint16_t _lcd_init_list[] = {


            SPFD54124B_CMD_SLPOUT,
            SPFD54124B_CMD_MADCTR,SPFD54124B_SEND_DATA | SPFD54124B_CMD_MADCTR_ML,
        #ifdef RGB12    
            SPFD54124B_CMD_COLMOD, SPFD54124B_SEND_DATA | SPFD54124B_CMD_COLMOD_MCU12bit,
        #else
            SPFD54124B_CMD_COLMOD, SPFD54124B_SEND_DATA | SPFD54124B_CMD_COLMOD_MCU16bit,
        #endif
            
        #if LCDTYPE==0    
            SPFD54124B_CMD_INVON,
        #else   
            SPFD54124B_CMD_INVOFF,
        #endif
            SPFD54124B_CMD_DISPON
            //SPFD54124B_CMD_NORON
        };

        ResetSPI();
        delay(100);
        
        uint8_t size = 5;


        const uint16_t* list = &_lcd_init_list[0];
        size = sizeof(_lcd_init_list) / sizeof(_lcd_init_list[0]);
        while (size--){
            writeany(*list++);
        }
      
        CS_OFF
    }
    #endif
    #if INCLUDE_ST7565
    void ResetST7565(void){
      pinMode(_sda2,OUTPUT);
      ResetSPI();
      // LCD bias select
      writeCommand(CMD_SET_BIAS_7);
      // ADC select
      writeCommand(CMD_SET_ADC_NORMAL);
      // SHL select
      writeCommand(CMD_SET_COM_REVERSE);
      // Initial display line
      writeCommand(CMD_SET_DISP_START_LINE);

      // turn on voltage converter (VC=1, VR=0, VF=0)
      writeCommand(CMD_SET_POWER_CONTROL | 0x4);
      // wait for 50% rising
      delay(50);

      // turn on voltage regulator (VC=1, VR=1, VF=0)
      writeCommand(CMD_SET_POWER_CONTROL | 0x6);
      // wait >=50ms
      delay(50);

      // turn on voltage follower (VC=1, VR=1, VF=1)
      writeCommand(CMD_SET_POWER_CONTROL | 0x7);
      // wait
      delay(10);

      // set lcd operating voltage (regulator resistor, ref voltage resistor)
      writeCommand(CMD_SET_RESISTOR_RATIO | 0x6);
      
      writeCommand(CMD_DISPLAY_ON);
      extern int lcd_contrast;
      writeCommand((lcd_contrast<0)?CMD_SET_DISP_REVERSE:CMD_SET_DISP_NORMAL);
      writeCommand(CMD_SET_ALLPTS_NORMAL);
      writeCommand(CMD_SET_VOLUME_FIRST);
      writeCommand(CMD_SET_VOLUME_SECOND | (abs(lcd_contrast) & 0x3f));
       
      CS_OFF
    }
    #endif
    
    inline void sendCommand(uint8_t command) __attribute__((always_inline)){
      if (_sda==255)return;
      initI2cIfNeccesary();
      Wire.beginTransmission(_address);
      Wire.write(0x80);
      Wire.write(command);
      Wire.endTransmission();
    }

    void initI2cIfNeccesary() {
      //if (_doI2cAutoInit) {
      	Wire.begin(this->_sda, this->_scl);
      //}
    }    
    void clear(void) {
      memset(buffer, 0, displayBufferSize);
    }
    void setColor(OLEDDISPLAY_COLOR color) {
      this->color = color;
    }
    void setFont(const uint8_t* font);
    void drawString(int16_t xMove, int16_t yMove, String strUser);
    void drawRect(int16_t x, int16_t y, int16_t width, int16_t height);
    void fillRect(int16_t xMove, int16_t yMove, int16_t width, int16_t height);
    void drawHorizontalLine(int16_t x, int16_t y, int16_t length);
    void drawVerticalLine(int16_t x, int16_t y, int16_t length);
    char* utf8ascii(String str);
    void drawStringInternal(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth) ;
    uint16_t getStringWidth(const char* text, uint16_t length);
    uint16_t getStringWidth(String strUser);
    void drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData);


};

