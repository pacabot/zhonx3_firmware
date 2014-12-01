/**************************************************************************/
/*! 
    @file     ssd1306.h
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10
*/
/**************************************************************************/
#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "peripherals/display/smallfonts.h"
#include "peripherals/display/pictures.h"
#include "peripherals/expander/pcf8574.h"

#define SSD1306_LCDWIDTH                  (128)
#define SSD1306_LCDHEIGHT                 (64)
#define SSD1306_LCDPAGEHEIGHT             (8)   //8 lines per page

// Commands
#define SSD1306_SETCONTRAST               0x81
#define SSD1306_DISPLAYALLON_RESUME       0xA4
#define SSD1306_DISPLAYALLON              0xA5
#define SSD1306_NORMALDISPLAY             0xA6
#define SSD1306_INVERTDISPLAY             0xA7
#define SSD1306_DISPLAYOFF                0xAE
#define SSD1306_DISPLAYON                 0xAF
#define SSD1306_SETDISPLAYOFFSET          0xD3
#define SSD1306_SETCOMPINS                0xDA
#define SSD1306_SETVCOMDETECT             0xDB
#define SSD1306_SETDISPLAYCLOCKDIV        0xD5
#define SSD1306_SETPRECHARGE              0xD9
#define SSD1306_SETMULTIPLEX              0xA8
#define SSD1306_SETLOWCOLUMN              0x00
#define SSD1306_SETHIGHCOLUMN             0x10
#define SSD1306_SETSTARTLINE              0x40
#define SSD1306_MEMORYMODE                0x20
#define SSD1306_COMSCANINC                0xC0
#define SSD1306_COMSCANDEC                0xC8
#define SSD1306_SEGREMAP                  0xA0
#define SSD1306_CHARGEPUMP                0x8D
#define SSD1306_EXTERNALVCC               0x1
#define SSD1306_SWITCHCAPVCC              0x2

// Initialisation/Config Prototypes
void ssd1306Init(unsigned char vccstate);
void ssd1306DrawPixel(unsigned char x, unsigned char y);
void ssd1306ClearPixel(unsigned char x, unsigned char y);
void ssd1306InvertPixel(unsigned char x, unsigned char y);
unsigned char ssd1306GetPixel(unsigned char x, unsigned char y);
void ssd1306ClearScreen(void);
void ssd1306Refresh(void);
void ssd1306DrawString(unsigned int x, unsigned int y, const char *text, const FONT_DEF *font);
void ssd1306PrintInt(unsigned int x,unsigned int y, const char *text, int val, const FONT_DEF *font);
void ssd1306ShiftFrameBuffer(unsigned char height);
void ssd1306DrawBmp(const unsigned char *bitmap, unsigned char x, unsigned char y, unsigned char w, unsigned char h);
void ssd1306DrawCircle(unsigned char x0, unsigned char y0, unsigned char r);
void ssd1306FillCircle(unsigned char x0, unsigned char y0, unsigned char r);
void ssd1306ClearCircle(unsigned char x0, unsigned char y0, unsigned char r);
void ssd1306FillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h);
void ssd1306ClearRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h);
void ssd1306DrawRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h);
void ssd1306InvertArea(unsigned char x, unsigned char y, unsigned char w, unsigned char h);
void ssd1306DrawLine(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1);
void ssd1306ProgressBar(unsigned char x, unsigned char y, unsigned char state);
#endif
