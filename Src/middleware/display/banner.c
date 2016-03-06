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

static void SetBeeperIcon(int charge_level);
static void SetBatIcon(int charge_level);
void bannerExample(void);

void bannerSetIcon(enum iconType icon, int val)
{
	switch (icon)
	{
	case LINESENSORS:
		ssd1306ClearRect(38, 0, 13, 7);
		if (val != FALSE)
			ssd1306DrawBmp(IRLine_Icon, 38, -1, 13, 8);
		ssd1306Refresh();
		break;
	case TELEMETERS:
		ssd1306ClearRect(55, 0, 13, 7);
		if (val != FALSE)
			ssd1306DrawBmp(IR_Icon3, 55, -1, 13, 8);
		ssd1306Refresh();
		break;
	case GYROMETER:
		ssd1306ClearRect(72, 0, 12, 7);
		if (val != FALSE)
			ssd1306DrawBmp(gyro_Icon1, 72, -1, 12, 8);
		ssd1306Refresh();
		break;
	case BEEPER:
		SetBeeperIcon(val);
		ssd1306Refresh();
		break;
	case BLUETOOTH:
		ssd1306ClearRect(105, 0, 5, 7);
		if (val != FALSE)
			ssd1306DrawBmp(BT_Icon, 105, -1,  5, 8);
		ssd1306Refresh();
		break;
	case BATTERY:
		SetBatIcon(val);
		break;
	case USB:
		ssd1306ClearRect(115, 0, 13, 7);
		if (val != FALSE)
			ssd1306DrawBmp(USB_Icon, 115, -1, 13, 8);
		ssd1306Refresh();
		break;
	}
}

static void SetBeeperIcon(int volume)
{
	if(volume == 0)
	{
		ssd1306ClearRect(89, 0, 11, 7);
		return;
	}
	ssd1306DrawBmp(beeper_Icon, 89, -1, 11, 8);

	if (volume > 100)
		volume = 100;
	if (volume < 1)
		volume = 1;

	volume = volume * 3 / 100;

	switch (volume)
	{
	case 0:
		ssd1306ClearRect(89 + 11 - 6, 0, 6, 7);
		break;
	case 1:
		ssd1306ClearRect(89 + 11 - 4, 0, 4, 7);
		break;
	case 2:
		ssd1306ClearRect(89 + 11 - 2, 0, 2, 7);
		break;
	}
}

static void SetBatIcon(int charge_level)
{
	static char old_level = 0;

	if (charge_level > 100)
		charge_level = 100;
	if (charge_level < 1)
		charge_level = 1;

	charge_level = charge_level * 12 / 100;

	if (charge_level == old_level)
		return;

	old_level = charge_level;

	ssd1306ClearRect(115, 0, 13, 7);
	ssd1306DrawBmp(bat_Icon, 115, -1, 13, 8);

	if (charge_level >= 1)
		ssd1306DrawPixel(117,4);
	if (charge_level >= 2){
		ssd1306DrawPixel(118,4);
		ssd1306DrawPixel(117,3);}
	if (charge_level >= 3){
		ssd1306DrawPixel(118,3);
		ssd1306DrawPixel(117,2);}
	if (charge_level >= 4)
		ssd1306DrawPixel(118,2);
	if (charge_level >= 5)
		ssd1306DrawPixel(120,4);
	if (charge_level >= 6){
		ssd1306DrawPixel(121,4);
		ssd1306DrawPixel(120,3);}
	if (charge_level >= 7){
		ssd1306DrawPixel(121,3);
		ssd1306DrawPixel(120,2);}
	if (charge_level >= 8)
		ssd1306DrawPixel(121,2);
	if (charge_level >= 9)
		ssd1306DrawPixel(123,4);
	if (charge_level >= 10){
		ssd1306DrawPixel(124,4);
		ssd1306DrawPixel(123,3);}
	if (charge_level >= 11){
		ssd1306DrawPixel(124,3);
		ssd1306DrawPixel(123,2);}
	if (charge_level >= 12)
		ssd1306DrawPixel(124,2);

	ssd1306Refresh();
}

void bannerExample(void)
{
	bannerSetIcon(LINESENSORS, TRUE);
	bannerSetIcon(TELEMETERS, TRUE);
	bannerSetIcon(GYROMETER, TRUE);
	bannerSetIcon(BLUETOOTH, TRUE);
	bannerSetIcon(USB, TRUE);

	for (int i = 0; i <= 100; i++)
	{
		bannerSetIcon(BEEPER, i);
		bannerSetIcon(BATTERY, i);
		HAL_Delay(50);
	}
	HAL_Delay(5000);
}
