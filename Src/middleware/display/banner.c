/*
 *  banner.c
 *
 *  Created on: 11 november 2015
 *      Author: patrick
 */
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

/* Application declarations */
#include "middleware/display/icons.h"
#include "middleware/display/pictures.h"

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"


/* Declarations for this module */
#include "middleware/display/banner.h"

void bannerExample(void);
#define SSD1306_BANNER_REFRESH_FPS		1

void banner_IT(void)
{
	static char cnt = 0;
	cnt++;

	if ((cnt % (LOW_TIME_FREQ / SSD1306_BANNER_REFRESH_FPS)) == TRUE)
		bannerExample();

	if (cnt > LOW_TIME_FREQ)
		cnt = 0;
}

void bannerExample(void)
{
	static char i = 0;
	i++;

	ssd1306ClearScreen(BANNER_AREA);

	if (i > 12)
	{
		ssd1306DrawBmp(USB_Icon, 115, 0, 13, 8);
	}
	else
	{
		ssd1306DrawBmp(bat_Icon, 115, 0, 13, 8);
		if (i >= 1)
		{
			ssd1306DrawPixel(117,5);
		}

		if (i >= 2)
		{
			ssd1306DrawPixel(118,5);
			ssd1306DrawPixel(117,4);
		}

		if (i >= 3)
		{
			ssd1306DrawPixel(118,4);
			ssd1306DrawPixel(117,3);
		}

		if (i >= 4)
		{
			ssd1306DrawPixel(118,3);
		}

		if (i >= 5)
		{
			ssd1306DrawPixel(120,5);
		}

		if (i >= 6)
		{
			ssd1306DrawPixel(121,5);
			ssd1306DrawPixel(120,4);
		}

		if (i >= 7)
		{
			ssd1306DrawPixel(121,4);
			ssd1306DrawPixel(120,3);
		}

		if (i >= 8)
		{
			ssd1306DrawPixel(121,3);
		}

		if (i >= 9)
		{
			ssd1306DrawPixel(123,5);
		}

		if (i >= 10)
		{
			ssd1306DrawPixel(124,5);
			ssd1306DrawPixel(123,4);
		}

		if (i >= 11)
		{
			ssd1306DrawPixel(124,4);
			ssd1306DrawPixel(123,3);
		}

		if (i >= 12)
		{
			ssd1306DrawPixel(124,3);
		}

		if (i > 40)
			ssd1306DrawBmp(BT_Icon,  105, 0,  5, 8);
		if (i < 50)
			ssd1306DrawBmp(beeper_Icon, 89, 0, 11, 8);
		if (i > 10)
			ssd1306DrawBmp(gyro_Icon1, 72, 0, 12, 8);
		if (i < 48)
			ssd1306DrawBmp(IR_Icon3, 55, 0, 13, 8);
		if (i > 24)
			ssd1306DrawBmp(IRLine_Icon, 38, 0, 13, 8);

		ssd1306PrintInt(10, 0, "", (char) i, &Font_3x6);
		ssd1306PrintInt(0, 0, "", 0, &Font_3x6);
		ssd1306DrawString(8, 0, ":", &Font_3x6);

		ssd1306Refresh(BANNER_AREA);
	}
}
