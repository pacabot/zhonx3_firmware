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
#include "peripherals/expander/pcf8574.h"

#include "stdbool.h"

/* extern variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

void Expander_Led_State(char led, char val)
{
	switch (led)
	{
	case 1:
		ExpanderSetbit(4, reverse_bit(val));
		break;
	case 2:
		ExpanderSetbit(5, reverse_bit(val));
		break;
	case 3:
		ExpanderSetbit(6, reverse_bit(val));
		break;
	}
}

void Expander_Joy_State(char led, char val)
{
//	ExpanderGetbit(char, pin);
}

//Send DATA
static void sendData(uint8_t aTxBuffer)
{
    // I2C
	HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)64, (uint8_t*)&aTxBuffer, 1);
}

//Send DATA
static char getData(void)
{
	// I2C
	uint8_t aRxBuffer[1];

	HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)65, (uint8_t *)aRxBuffer, 1);// 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
	return aRxBuffer[0];
}

void ExpanderSetbit(char pin, char val)
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

char ExpanderGetbit(char pin)
{
	return getData();
}

void ExpanderReset(void)
{
	sendData(0xf);
}
