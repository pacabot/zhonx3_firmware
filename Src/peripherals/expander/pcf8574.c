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
#include "peripherals/bluetooth/bluetooth.h"

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
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)64, (uint8_t*)&aTxBuffer, 1);

	//	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)64, (uint8_t *)&aTxBuffer, 1, 1000) != HAL_OK)
	//	{
	//		/* Error_Handler() function is called when Timout error occurs.
	//	       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
	//	       Master restarts communication */
	//		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	//		{
	//			bluetoothPrintf("I2C setExpander error \r\n");
	//		}
	//	}
}

//get DATA
static char getData(void)
{
	// I2C
	static uint8_t aRxBuffer;

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	while (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)65, (uint8_t *)&aRxBuffer, 1, 1000) != HAL_OK)
	{
		/* Error_Handler() function is called when Timout error occurs.
	       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
	       Master restarts communication */
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			bluetoothPrintf("I2C getExpander error \r\n");
		}
	}

	return aRxBuffer;
}

void expanderInit(void)
{
	sendData(0xFF);
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

void expanderLedState(char led, char val)
{
	switch (led)
	{
	case 3:
		expanderSetbit(4, reverse_bit(val));
		break;
	case 2:
		expanderSetbit(5, reverse_bit(val));
		break;
	case 1:
		expanderSetbit(6, reverse_bit(val));
		break;
	}
}

char expanderJoyState(void)
{
	switch (~(getData() | 0xF0))
	{
	case 4:
		//		bluetoothPrintf("Joystick UP\n");
		return JOY_UP;
		break;
	case 1:
		//		bluetoothPrintf("Joystick DOWN\n");
		return JOY_DOWN;
		break;
	case 8:
		//		bluetoothPrintf("Joystick LEFT\n");
		return JOY_LEFT;
		break;
	case 2:
		//		bluetoothPrintf("Joystick RIGHT\n");
		return JOY_RIGHT;
		break;
	default :
		//bluetoothPrintf("Joystick ERROR\n");
		return 0;
	}
}
char expanderJoyFiltered(void)
{
	if (antiBounceJoystick2(expanderJoyState())==DONE)
		return expanderJoyState();
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

		ssd1306ClearScreen(MAIN_AREA);
		ssd1306DrawCircle(60,20, 3);
		ssd1306DrawCircle(60,40, 3);
		ssd1306DrawCircle(50,30, 3);
		ssd1306DrawCircle(70,30, 3);
		switch (state)
		{
		case JOY_UP:
			ssd1306FillCircle(60,20, 3);
			break;
		case JOY_DOWN:
			ssd1306FillCircle(60,40, 3);
			break;
		case JOY_LEFT:
			ssd1306FillCircle(50,30, 3);
			break;
		case JOY_RIGHT:
			ssd1306FillCircle(70,30, 3);
			break;
		}
		ssd1306Refresh();
	}
	ssd1306FillCircle(50,30, 3);
	ssd1306Refresh();
	HAL_Delay(1000);
	antiBounceJoystick();
}

void expenderLedTest ()
{
	while(expanderJoyState()!=JOY_LEFT)
	{
		expanderLedState(1,1);
		HAL_Delay(250);
		expanderLedState(2,1);
		HAL_Delay(250);
		expanderLedState(3,1);
		HAL_Delay(250);
		expanderLedState(1,0);
		HAL_Delay(250);
		expanderLedState(2,0);
		HAL_Delay(250);
		expanderLedState(3,0);
		HAL_Delay(250);
	}
	antiBounceJoystick();
}
