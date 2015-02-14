/**************************************************************************/
/*!
    @file     expander.c
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Driver for expander PCF8574
 */
/**************************************************************************/
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "config/basetypes.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"

#include "stdbool.h"

/* extern variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

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

	HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)65, (uint8_t *)aRxBuffer, 1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
	return aRxBuffer[0];
}

void expanderSetbit(char pin, char val)
{
	if(val == true)
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
	return getData();
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
	int key = 0;
	key = ~(getData() | 0xFFFFFFF0);

	switch (key)
	{
	case 4:
		return UP;
		break;
	case 1:
		return DOWN;
		break;
	case 8:
		return LEFT;
		break;
	case 2:
		return RIGHT;
		break;
	case 0:
		return 0;
		break;
	default :
		return SEVERAL;
	}
	return 0;
}

void joystickTest(void)
{
	int state;

	while(expanderJoyState()!=LEFT)
	{
		state = expanderJoyState();

		ssd1306ClearScreen();
		ssd1306DrawCircle(60,10, 3);
		ssd1306DrawCircle(60,30, 3);
		ssd1306DrawCircle(50,20, 3);
		ssd1306DrawCircle(70,20, 3);

		switch (state)
		{
		case UP:
			ssd1306FillCircle(60,10, 3);
			break;
		case DOWN:
			ssd1306FillCircle(60,30, 3);
			break;
		case LEFT:
			ssd1306FillCircle(50,20, 3);
			break;
		case RIGHT:
			ssd1306FillCircle(70,20, 3);
			break;
		}

		ssd1306Refresh();
	}
	ssd1306FillCircle(50,20, 3);
	ssd1306Refresh();
	HAL_Delay(1000);
	antiBounceJoystick();
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
