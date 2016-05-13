/**************************************************************************/
/*!
 @file     ssd1306.c
 @author   PLF
 @date     08 March 2015
 @version  1.00

 Driver for 128x64 OLED display based on the SSD1306 controller.

 This driver is based on the SSD1306 Library from Limor Fried
 (Adafruit Industries) at: https://github.com/adafruit/SSD1306  
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

/* Application declarations */

/* Peripheral declarations */
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */
#include "middleware/display/icons.h"
#include "middleware/display/pictures.h"

/* Declarations for this module */
#include "peripherals/display/ssd1306.h"

#define SSD1306_CMDSIZE					1

#define SSD1306_BANNERSIZE			    (SSD1306_LCDWIDTH)
#define SSD1306_MAINSIZE				((SSD1306_LCDWIDTH * SSD1306_LCDPAGEHEIGHT) - SSD1306_BANNERSIZE)

// Commands
#define SSD1306_SETCONTRAST             0x81
#define SSD1306_DISPLAYALLON_RESUME     0xA4
#define SSD1306_DISPLAYALLON            0xA5
#define SSD1306_NORMALDISPLAY           0xA6
#define SSD1306_INVERTDISPLAY           0xA7
#define SSD1306_DISPLAYOFF          	0xAE
#define SSD1306_DISPLAYON               0xAF
#define SSD1306_SETDISPLAYOFFSET        0xD3
#define SSD1306_SETCOMPINS              0xDA
#define SSD1306_SETVCOMDETECT           0xDB
#define SSD1306_SETDISPLAYCLOCKDIV      0xD5
#define SSD1306_SETPRECHARGE            0xD9
#define SSD1306_SETMULTIPLEX            0xA8
#define SSD1306_SETPAGESTART		  	0xB0
#define SSD1306_SETLOWCOLUMN            0x00
#define SSD1306_SETHIGHCOLUMN           0x10
#define SSD1306_SETSTARTLINE            0x40
#define SSD1306_MEMORYMODE              0x20
#define SSD1306_COMSCANINC              0xC0
#define SSD1306_COMSCANDEC              0xC8
#define SSD1306_SEGREMAP                0xA0
#define SSD1306_CHARGEPUMP              0x8D
#define SSD1306_EXTERNALVCC             0x01
#define SSD1306_SWITCHCAPVCC            0x02

/* extern variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
//extern DMA_HandleTypeDef hdma_i2c1_tx;

/* Private variables ---------------------------------------------------------*/
//static unsigned char banner_aera_buffer[SSD1306_BANNERSIZE];
static unsigned char display_buffer[SSD1306_CMDSIZE + SSD1306_MAINSIZE + SSD1306_BANNERSIZE];

/**************************************************************************/
/*  Private Function Prototypes                                           */
/**************************************************************************/
static void ssd1306DrawChar(unsigned int x, unsigned int y, unsigned char c, const FONT_DEF *font);

/**************************************************************************/
/* Macros                                                                 */
/**************************************************************************/
#define swap(a, b){ \
        unsigned char c; \
        c = a; \
        a = b; \
        b = c; \
}

//Send CMD
static void CMD(uint8_t c)
{
    // I2C
    uint8_t control = 0x00;   // Co = 0, D/C = 0

    uint8_t aTxBuffer[2]; // = {control, c};
    aTxBuffer[0] = control;
    aTxBuffer[1] = c;

    //	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    //	{
    //	}
    HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t) 120, (uint8_t*) aTxBuffer, 2);  //, 1000);
}

//Send DATA
static void DATA(uint8_t c[], uint16_t size)
{
    // I2C
    //	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    //	{
    //	}
    HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t) 120, (uint8_t*) c, size); //, 1000);
}

/**************************************************************************/
/*!
 @brief  Draws a single graphic character using the supplied font
 */
/**************************************************************************/
static void ssd1306DrawChar(unsigned int x, unsigned int y, unsigned char c, const FONT_DEF *font)
{
    unsigned int xoffset;
    unsigned char yoffset;
    unsigned char col;
    unsigned char column[font->u8Width];

    // Capitalize character if lower case is not available
    if ((c > font->u8LastChar) && (font->u8LastChar < 128))
    {
        c -= 32;
    }

    // Check if the requested character is available
    if ((c >= font->u8FirstChar) && (c <= font->u8LastChar))
    {
        // Retrieve appropriate columns from font data
        for (col = 0; col < font->u8Width; col++)
        {
            column[col] = font->au8FontTable[((c - 32) * font->u8Width) + col]; // Get first column of appropriate character
        }
    }
    else
    {
        // Requested character is not available in this font ... send a space instead
        for (col = 0; col < font->u8Width; col++)
        {
            column[col] = 0xFF;    // Send solid space
        }
    }

    // Render each column
    for (xoffset = 0; xoffset < font->u8Width; xoffset++)
    {
        for (yoffset = 0; yoffset < (font->u8Height + 1); yoffset++)
        {
            unsigned char bit = 0x00;
            bit = (column[xoffset] << (8 - (yoffset + 1))); // Shift current row bit left
            bit = (bit >> 7); // Shift current row but right (results in 0x01 for black, and 0x00 for white)
            if (bit)
            {
                ssd1306DrawPixel(x + xoffset, y + yoffset);
            }
        }
    }
}

/**************************************************************************/
/* Public Methods                                                         */
/**************************************************************************/

/**************************************************************************/
/*! 
 @brief Initialises the SSD1306 LCD display
 */
/**************************************************************************/
void ssd1306Init(unsigned char vccstate)
{
    // Reset the LCD
    expanderSetbit(7, 1);
    HAL_Delay(50);
    expanderSetbit(7, 0);
    HAL_Delay(50);
    expanderSetbit(7, 1);
    HAL_Delay(50);

    // Initialization sequence
    CMD(SSD1306_DISPLAYOFF);                    // 0xAE
    CMD(SSD1306_SETLOWCOLUMN | 0x0);            // low col = 0
    CMD(SSD1306_SETHIGHCOLUMN | 0x0);           // hi col = 0
    CMD(SSD1306_SETSTARTLINE | 0x0);            // line #0
    CMD(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC)
    {
        CMD(0x9F);								// contrast (0-255)
    }
    else
    {
        CMD(254);								// contrast (0-255)
    }
    CMD(SSD1306_NORMALDISPLAY);                 // 0xA6 NORMAL MODE (A7 for inverse display)
    //	CMD(SSD1306_INVERTDISPLAY);             // 0xA6 NORMAL MODE (A7 for inverse display)
    CMD(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    CMD(SSD1306_SETMULTIPLEX);                  // 0xA8
    CMD(0x3F);                                  // 0x3F 1/64 duty
    CMD(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    CMD(0x0);                                   // no offset
    CMD(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    CMD(0b10000000);                            // the suggested ratio 0x80
    CMD(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC)
    {
        CMD(0x22);
    }
    else
    {
        CMD(0xF1);
    }
    CMD(SSD1306_SETCOMPINS);                    // 0xDA
    CMD(0x12);                                  // disable COM left/right remap
    CMD(SSD1306_SETVCOMDETECT);                 // 0xDB
    CMD(0x40);                                  // 0x20 is default?
    CMD(SSD1306_MEMORYMODE);                    // 0x20
    CMD(0x00);                                  // 0x0 act like ks0108

    //	CMD(SSD1306_SEGREMAP | 0x1);			//   For
    //	CMD(SSD1306_COMSCANDEC);				// Normal display
    CMD(SSD1306_SEGREMAP);					    //   For rotate
    CMD(SSD1306_COMSCANINC);					// 180 degrees display

    CMD(SSD1306_CHARGEPUMP);                    //0x8D
    if (vccstate == SSD1306_EXTERNALVCC)
    {
        CMD(0x10);
    }
    else
    {
        CMD(0x14);
    }

    // Enabled the OLED panel
    CMD(SSD1306_DISPLAYON);
}

/**************************************************************************/
/*! 
 @brief Draws a single pixel in image buffer

 @param[in]  x
 The x position (0..127)
 @param[in]  y
 The y position (0..63)
 */
/**************************************************************************/
void ssd1306DrawPixel(unsigned char x, unsigned char y)
{
    if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
        return;

    display_buffer[x + ((y / 8) * SSD1306_LCDWIDTH) + SSD1306_CMDSIZE] |= (1 << y % 8);
}

/**************************************************************************/
/*!
 @brief Clears a single pixel in image buffer

 @param[in]  x
 The x position (0..127)
 @param[in]  y
 The y position (0..63)
 */
/**************************************************************************/
void ssd1306ClearPixel(unsigned char x, unsigned char y)
{
    if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
        return;

    display_buffer[x + ((y / 8) * SSD1306_LCDWIDTH) + SSD1306_CMDSIZE] &= ~(1 << y % 8);
}

/**************************************************************************/
/*!
 @brief Inverts a single pixel in image buffer

 @param[in]  x
 The x position (0..127)
 @param[in]  y
 The y position (0..63)
 */
/**************************************************************************/
void ssd1306InvertPixel(unsigned char x, unsigned char y)
{
    if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
        return;

    display_buffer[x + ((y / 8) * SSD1306_LCDWIDTH) + SSD1306_CMDSIZE] ^= (1 << y % 8);
}

/**************************************************************************/
/*! 
 @brief Gets the value (1 or 0) of the specified pixel from the buffer

 @param[in]  x
 The x position (0..127)
 @param[in]  y
 The y position (0..63)

 @return     1 if the pixel is enabled, 0 if disabled
 */
/**************************************************************************/
unsigned char ssd1306GetPixel(unsigned char x, unsigned char y)
{
    if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
        return 0;

    return display_buffer[(x + (y / 8) * SSD1306_LCDWIDTH) + SSD1306_CMDSIZE] & (1 << y % 8) ? 1 : 0;
}

/**************************************************************************/
/*! 
 @brief Clears the screen
 */
/**************************************************************************/
void ssd1306ClearScreen(enum refreshTypeEnum clearType)
{
    switch (clearType)
    {
        case BANNER_AREA:
            memset(display_buffer + SSD1306_CMDSIZE, 0, SSD1306_BANNERSIZE);
            break;
        case MAIN_AREA:
            memset(display_buffer + (SSD1306_BANNERSIZE + SSD1306_CMDSIZE), 0, SSD1306_MAINSIZE);
            break;
        case ALL_AREA:
            memset(display_buffer + SSD1306_CMDSIZE, 0, SSD1306_BANNERSIZE);
            memset(display_buffer + (SSD1306_BANNERSIZE + SSD1306_CMDSIZE), 0, SSD1306_MAINSIZE);
            break;
    }
}

/**************************************************************************/
/*!
 @brief wait I2C_STATE_READY
 */
/**************************************************************************/
void ssd1306WaitReady(void)
{
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
        HAL_I2C_MspDeInit(&hi2c1);
        HAL_I2C_MspInit(&hi2c1);
    }
}

/**************************************************************************/
/*!
 @brief Renders the contents of the pixel buffer on the LCD
 */
/**************************************************************************/
void ssd1306Refresh(void)
{
    ssd1306DrawDashedLine(0, 8, 128, 8);
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
        return;
    display_buffer[0] = 0x40;
    DATA(display_buffer,
         SSD1306_MAINSIZE + SSD1306_BANNERSIZE + SSD1306_CMDSIZE);
}

/**************************************************************************/
/*!
 @brief  Draws a string using the supplied font data.

 @param[in]  x
 Starting x co-ordinate
 @param[in]  y
 Starting y co-ordinate
 @param[in]  text
 The string to render
 @param[in]  font
 Pointer to the FONT_DEF to use when drawing the string

 @section Example

 @code 

 #include "drivers/lcd/bitmap/ssd1306/ssd1306.h"
 #include "drivers/lcd/smallfonts.h"

 // Configure the pins and initialise the LCD screen
 ssd1306Init();

 // Render some text on the screen
 ssd1306DrawString(1, 10, "5x8 System", Font_System5x8);
 ssd1306DrawString(1, 20, "7x8 System", Font_System7x8);

 // Refresh the screen to see the results
 ssd1306Refresh();

 @endcode
 */
/**************************************************************************/
void ssd1306DrawString(unsigned int x, unsigned int y, const char *text, const FONT_DEF *font)
{
    unsigned char l;
    for (l = 0; l < strlen((const char *) text); l++)
    {
        // Do not send non-printable characters
        if ((text[l] <= 0x0E) || (text[l] == 0x7F))
        {
            continue;
        }
        ssd1306DrawChar(x + (l * (font->u8Width + 1)), y, text[l], font);
    }
}

void ssd1306DrawStringAtLine(unsigned int x, unsigned int line, const char *text, const FONT_DEF *font)
{
    ssd1306DrawString(x, line * LINE_SPACING + HEAD_MARGIN, text, font);
}

/**************************************************************************/
//Print a text and value.
void ssd1306PrintInt(unsigned int x, unsigned int y, const char *text, unsigned int val, const FONT_DEF *font)
{
    char str[10];
    unsigned int nb_char;

    nb_char = strlen(text);

    //	itoa(val, str, 5, 10);  // 10 for decimal 2 for bin 16 for hex

    sprintf(str, "%d     ", val);
    ssd1306DrawString(x, y, text, font);
    //i = (x + ((nb_char + 1) * font->u8Width));
    ssd1306DrawString((x + ((nb_char + 1) * font->u8Width)), y, str, font);
}

void ssd1306PrintIntAtLine(unsigned int x, unsigned int line, const char *text, unsigned int val, const FONT_DEF *font)
{
    ssd1306PrintInt(x, line * LINE_SPACING + HEAD_MARGIN, text, val, font);
}

void ssd1306Printf(int x, int y, const FONT_DEF *font, const char *format, ...)
{
    char temp_buffer[43];
    va_list va_args;

    va_start(va_args, format);
    vsnprintf(temp_buffer, 43, format, va_args);
    va_end(va_args);

    ssd1306DrawString(x, y, (char *) temp_buffer, font);
}

void ssd1306PrintfAtLine(int x, int line, const FONT_DEF *font, const char *format, ...)
{
    int y = line * LINE_SPACING + HEAD_MARGIN;
    char temp_buffer[43];
    va_list va_args;

    va_start(va_args, format);
    vsnprintf(temp_buffer, 43, format, va_args);
    va_end(va_args);

    ssd1306DrawString(x, y, (char *) temp_buffer, font);
}

/**************************************************************************/
/*!
 @brief  Shifts the contents of the frame buffer up the specified
 number of pixels

 @param[in]  height
 The number of pixels to shift the frame buffer up, leaving
 a blank space at the bottom of the frame buffer x pixels
 high

 @section Example

 @code 

 #include "drivers/lcd/bitmap/ssd1306/ssd1306.h"
 #include "drivers/lcd/smallfonts.h"

 // Configure the pins and initialise the LCD screen
 ssd1306Init();

 // Continually write some text, scrolling upward one line each time
 while (1)
 {
 // Shift the buffer up 8 pixels (adjust for font-height)
 ssd1306ShiftFrameBuffer(8);
 // Render some text on the screen with different fonts
 ssd1306DrawString(1, 56, "INSERT TEXT HERE", Font_System3x6);   // 3x6 is UPPER CASE only
 // Refresh the screen to see the results
 ssd1306Refresh();    
 // Wait a bit before writing the next line
 HAL_Delay(1000);
 }

 @endcode
 */
/**************************************************************************/
void ssd1306ShiftFrameBuffer(unsigned char height)
{
    if (height == 0)
        return;
    if (height >= SSD1306_LCDHEIGHT)
    {
        // Clear the entire frame buffer
        ssd1306ClearScreen(MAIN_AREA);
        return;
    }

    // This is horribly inefficient, but at least easy to understand
    // In a production environment, this should be significantly optimised

    unsigned char y, x;
    for (y = 0; y < SSD1306_LCDHEIGHT; y++)
    {
        for (x = 0; x < SSD1306_LCDWIDTH; x++)
        {
            if ((SSD1306_LCDHEIGHT - 1) - y > height)
            {
                // Shift height from further ahead in the buffer
                ssd1306GetPixel(x, y + height) ? ssd1306DrawPixel(x, y) : ssd1306ClearPixel(x, y);
            }
            else
            {
                // Clear the entire line
                ssd1306ClearPixel(x, y);
            }
        }
    }
}
/**************************************************************************/
/*
 Procedure to use ssd1306DrawBmp function :

 - make your picture with gimp
 - save to Bitmap file
 - open Image2LCD
 - chose into Scan Mode box => Data hor, Bite ver
 - chose into BitsPixel box => monochrome
 - check => Antitone pixel in byte
 - uncheck the rest
 - select => chose normal
 - chose into Output file type box => C array (*C)
 - save
 - open the file with SublimeText => select all => copy
 - paste in pictures_bmp.h
 */
/**************************************************************************/
void ssd1306DrawBmp(const unsigned char *bitmap, unsigned char x, unsigned char y, unsigned char w, unsigned char h)
{
    unsigned char i;
    unsigned char j;

    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            if (bitmap[i + (j / 8) * w] & 1 << (j % 8))
            {
                ssd1306DrawPixel(x + i, y + j);
            }
        }
    }
}
/**************************************************************************/
//draw a circle outline
void ssd1306DrawCircle(unsigned char x0, unsigned char y0, unsigned char r)
{
    signed char f = 1 - r;
    signed char ddF_x = 1;
    signed char ddF_y = -2 * r;
    signed char x = 0;
    signed char y = r;
    ssd1306DrawPixel(x0, y0 + r);
    ssd1306DrawPixel(x0, y0 - r);
    ssd1306DrawPixel(x0 + r, y0);
    ssd1306DrawPixel(x0 - r, y0);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306DrawPixel(x0 + x, y0 + y);
        ssd1306DrawPixel(x0 - x, y0 + y);
        ssd1306DrawPixel(x0 + x, y0 - y);
        ssd1306DrawPixel(x0 - x, y0 - y);

        ssd1306DrawPixel(x0 + y, y0 + x);
        ssd1306DrawPixel(x0 - y, y0 + x);
        ssd1306DrawPixel(x0 + y, y0 - x);
        ssd1306DrawPixel(x0 - y, y0 - x);
    }
}
/**************************************************************************/
void ssd1306FillCircle(unsigned char x0, unsigned char y0, unsigned char r)
{
    int i;
    signed char f = 1 - r;
    signed char ddF_x = 1;
    signed char ddF_y = -2 * r;
    signed char x = 0;
    signed char y = r;

    for (i = y0 - r; i <= y0 + r; i++)
    {
        ssd1306DrawPixel(x0, i);
    }

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        for (i = y0 - y; i <= y0 + y; i++)
        {
            ssd1306DrawPixel(x0 + x, i);
            ssd1306DrawPixel(x0 - x, i);
        }
        for (i = y0 - x; i <= y0 + x; i++)
        {
            ssd1306DrawPixel(x0 + y, i);
            ssd1306DrawPixel(x0 - y, i);
        }
    }
}
/**************************************************************************/
void ssd1306ClearCircle(unsigned char x0, unsigned char y0, unsigned char r)
{
    int i;
    signed char f = 1 - r;
    signed char ddF_x = 1;
    signed char ddF_y = -2 * r;
    signed char x = 0;
    signed char y = r;

    for (i = y0 - r; i <= y0 + r; i++)
    {
        ssd1306ClearPixel(x0, i);
    }

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        for (i = y0 - y; i <= y0 + y; i++)
        {
            ssd1306ClearPixel(x0 + x, i);
            ssd1306ClearPixel(x0 - x, i);
        }
        for (i = y0 - x; i <= y0 + x; i++)
        {
            ssd1306ClearPixel(x0 + y, i);
            ssd1306ClearPixel(x0 - y, i);
        }
    }
}
/**************************************************************************/
// filled rectangle
void ssd1306FillRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h)
{
    int i, j;
    // stupidest version - just pixels - but fast with internal buffer!
    for (i = x; i < x + w; i++)
    {
        for (j = y; j < y + h; j++)
        {
            ssd1306DrawPixel(i, j);
        }
    }
}

/**************************************************************************/
// Inverts area
void ssd1306InvertArea(unsigned char x, unsigned char y, unsigned char w, unsigned char h)
{
    int i, j;
    // stupidest version - just pixels - but fast with internal buffer!
    for (i = x; i < (x + w); i++)
    {
        for (j = y; j < (y + h); j++)
        {
            ssd1306InvertPixel(i, j);
        }
    }
}

/**************************************************************************/
void ssd1306ClearRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h)
{
    int i, j;
    // stupidest version - just pixels - but fast with internal buffer!
    for (i = x; i < x + w; i++)
    {
        for (j = y; j < y + h; j++)
        {
            ssd1306ClearPixel(i, j);
        }
    }
}
/**************************************************************************/
void ssd1306ClearRectAtLine(unsigned char x, unsigned char line, unsigned char w)
{
    int i, j;
    int y = line * LINE_SPACING + HEAD_MARGIN - 1;
    unsigned char h = LINE_SPACING - 1;
    // stupidest version - just pixels - but fast with internal buffer!
    for (i = x; i < x + w; i++)
    {
        for (j = y; j < y + h; j++)
        {
            ssd1306ClearPixel(i, j);
        }
    }
}

/**************************************************************************/
// draw a rectangle
void ssd1306DrawRect(unsigned char x, unsigned char y, unsigned char w, unsigned char h)
{
    int i;
    // stupidest version - just pixels - but fast with internal buffer!
    for (i = x; i < x + w; i++)
    {
        ssd1306DrawPixel(i, y);
        ssd1306DrawPixel(i, y + h - 1);
    }
    for (i = y; i < y + h; i++)
    {
        ssd1306DrawPixel(x, i);
        ssd1306DrawPixel(x + w - 1, i);
    }
}

void ssd1306DrawDashedLine(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1)
{
    uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1)
    {
        swap(x0, x1);
        swap(y0, y1);
    }

    uint8_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int8_t err = dx / 2;
    int8_t ystep;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (; x0 < x1; x0++)
    {
        if (steep)
        {
            if ((x0 % 2) == 0)
                ssd1306DrawPixel(y0, x0);
        }
        else
        {
            if ((x0 % 2) == 0)
                ssd1306DrawPixel(x0, y0);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

/**************************************************************************/
// bresenham's algorithm - thx wikpedia
void ssd1306DrawLine(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1)
{
    uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1)
    {
        swap(x0, x1);
        swap(y0, y1);
    }

    uint8_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int8_t err = dx / 2;
    int8_t ystep;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (; x0 < x1; x0++)
    {
        if (steep)
        {
            ssd1306DrawPixel(y0, x0);
        }
        else
        {
            ssd1306DrawPixel(x0, y0);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

/**************************************************************************/
// bresenham's algorithm - thx wikpedia
void ssd1306ClearLine(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1)
{
    uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        swap(x0, y0);
        swap(x1, y1);
    }

    if (x0 > x1)
    {
        swap(x0, x1);
        swap(y0, y1);
    }

    uint8_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int8_t err = dx / 2;
    int8_t ystep;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (; x0 < x1; x0++)
    {
        if (steep)
        {
            ssd1306ClearPixel(y0, x0);
        }
        else
        {
            ssd1306ClearPixel(x0, y0);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

void ssd1306ProgressBar(unsigned char x, unsigned char y, unsigned char state)
{
    ssd1306DrawRect(x, y, 104, 8);
    if ((state > 0) && (state <= 100))
        ssd1306FillRect(x + 2, y + 2, state, 4);

}

void ssd1306Test(void)
{
    int i;

    ssd1306DrawBmp(pacabot_Img, 1, 10, 128, 40);
    ssd1306Refresh();

    for (i = 0; i <= 100; i += 1)
    {
        ssd1306ProgressBar(10, 55, i);
        HAL_Delay(1);
        ssd1306Refresh();
    }

    ssd1306Refresh();
    HAL_Delay(500);
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Refresh();
    // miniature bitmap display
    ssd1306DrawCircle(40, 40, 20);
    //  ssd1306DrawCircle(50, 20, 10);
    ssd1306FillCircle(100, 50, 15);
    ssd1306DrawRect(110, 30, 15, 6);
    ssd1306FillRect(1, 60, 10, 20);
    ssd1306DrawDashedLine(5, 45, 70, 45);
    ssd1306DrawLine(70, 45, 20, 10);
    ssd1306Refresh();
    HAL_Delay(3000);
    ssd1306ClearScreen(MAIN_AREA);

    ssd1306DrawStringAtLine(13, 0, "Oled 128x64", &Font_8x8); // 3x6 is UPPER CASE only
    ssd1306Refresh();
    HAL_Delay(1500);
    ssd1306DrawStringAtLine(1, 1, "Driver for STM32f4xx", &Font_5x8); // 3x6 is UPPER CASE only
    ssd1306Refresh();
    HAL_Delay(500);
    ssd1306DrawStringAtLine(1, 2, "I2C mode", &Font_5x8); // 3x6 is UPPER CASE only
    ssd1306Refresh();
    HAL_Delay(1500);
    ssd1306DrawStringAtLine(10, 4, "BY PLF, PACABOT TEAM", &Font_3x6); // 3x6 is UPPER CASE only
    ssd1306Refresh();
    HAL_Delay(3000);

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Refresh();
}

