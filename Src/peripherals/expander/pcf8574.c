/**************************************************************************/
/*!
    @file     expander.c
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Driver for expander PCF8574
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
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/expander/pcf8574.h"
#define NORMAL_DELAY_REAPEAT 400
#define FAST_DELAY_REAPEAT 70
#define DONE 0
/* extern variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
unsigned int joy_activ_old_time;

//Send DATA
static void sendData(uint8_t aTxBuffer)
{
	// I2C
	HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)64, (uint8_t*)&aTxBuffer, 1);
}

//get DATA
static char getData(void)
{
	// I2C
	uint8_t aRxBuffer[1];

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
	HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)65, (uint8_t *)aRxBuffer, 1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}

	return aRxBuffer[0];
}

void expanderSetbit(char pin, char val)
{
	if(val == 1)
	{
		sendData(getData() | (0x1 << pin));
	}
	else
	{
		sendData(getData() & ~(0x1 << pin));
	}
}

char expanderGetbit(char pin)
{
	return getData(); //todo return bit
}

void expanderReset(void)
{
	sendData(0xF);
}

void expanderLedState(char led, char val)
{
	switch (led)
	{
		case 1:
			expanderSetbit(4, reverse_bit(val));
			break;
		case 2:
			expanderSetbit(5, reverse_bit(val));
			break;
		case 3:
			expanderSetbit(6, reverse_bit(val));
			break;
	}
}

char expanderJoyState(void)
{
	char key = 0;
	key = ~(getData() | 0xF0);
	switch (key)
	{
		case 4:
			return JOY_UP;
			break;
		case 1:
			return JOY_DOWN;
			break;
		case 8:
			return JOY_LEFT;
			break;
		case 2:
			return JOY_RIGHT;
			break;
		case 0:
			return 0;
			break;
		default :
			return JOY_SEVERAL;
	}
	return 0;
}
char expanderJoyFiltered(void)
{
	char joystick=expanderJoyState();
	if (antiBounceJoystick2(joystick)==DONE)
		return joystick;
	return 0;
}

void antiBounceJoystick(void)
{
	unsigned long int time_base = HAL_GetTick();
	do
	{
		if (expanderJoyState()!=0)
			time_base = HAL_GetTick();
	}while (time_base!=(HAL_GetTick()-20));
}

char antiBounceJoystick2(char arrow_type)
{
	static long time_base = 0;
	static char old_arrow_type = 0;
	static char fast_clic = false;
	long time=HAL_GetTick();
	if(old_arrow_type != arrow_type)
	{
		joy_activ_old_time = HAL_GetTick();
		old_arrow_type = arrow_type;
		time_base = time;
		fast_clic = false;
		return DONE;
	}
	else if((fast_clic == true) && ((time_base + FAST_DELAY_REAPEAT) < time))
	{
		time_base = time;
		return DONE;
	}
	else if (((time_base + NORMAL_DELAY_REAPEAT) < time) && (fast_clic == false))
	{
		time_base = time;
		fast_clic = true;
		return DONE;
	}

	return -1;
}

void joystickTest(void)
{
	int state;

	while(expanderJoyState()!=JOY_LEFT)
	{
		state = expanderJoyState();

		ssd1306ClearScreen(MAIN_AERA);
		ssd1306DrawCircle(60,10, 3);
		ssd1306DrawCircle(60,30, 3);
		ssd1306DrawCircle(50,20, 3);
		ssd1306DrawCircle(70,20, 3);
		switch (state)
		{
			case JOY_UP:
				ssd1306FillCircle(60,10, 3);
				break;
			case JOY_DOWN:
				ssd1306FillCircle(60,30, 3);
				break;
			case JOY_LEFT:
				ssd1306FillCircle(50,20, 3);
				break;
			case JOY_RIGHT:
				ssd1306FillCircle(70,20, 3);
				break;
		}

		ssd1306Refresh(MAIN_AERA);
	}
	ssd1306FillCircle(50,20, 3);
	ssd1306Refresh(MAIN_AERA);
	HAL_Delay(1000);
	antiBounceJoystick();
}

void expenderLedTest ()
{
//	char i=0;
//	while(expanderJoyState()!=JOY_LEFT)
//	{
//		if ((HAL_GetTick() % 200) == 0)
//		{
//			expanderSetbit(i,0);
//			i = (i % 3)+1;
//			expanderLedState(i,1);
//			ssd1306ClearScreen(MAIN_AERA);
//			ssd1306Printf(0,0,&Font_5x8,"i : %d", i);
//			ssd1306Refresh(MAIN_AERA);
//		}
//	}
//	for (i=1; i<4; i++)
//	{
//		expanderLedState(i,1);
//	}
//	HAL_Delay(500);
//	for (i=1; i<4; i++)
//	{
//		expanderLedState(i,0);
//	}
	expanderLedState(2,1);
	joystickTest();
	HAL_Delay(500);
}
