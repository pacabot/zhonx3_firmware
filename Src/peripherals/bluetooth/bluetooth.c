/**************************************************************************/
/*!
    @file    Bluetooth.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "util/itoa.h"

extern UART_HandleTypeDef huart3;

void Bluetooth_Init(void)
{


}

void printString(const char *text)
{

//	unsigned char l;
//	for (l = 0; l < strlen(text); l++)
//	{
//		ssd1306DrawChar(x + (l * (font->u8Width + 1)), y, text[l], font);
//	}
	  HAL_UART_Transmit(&huart3, "hello ZHONX III", 16, 1000);
}

void printInt(unsigned int val)
{
	  unsigned char str[10];

	  sprintf(str, "%d      ", val);
///	  itoa(val, aTxBuffer, 5, 10);  // 10 for decimal 2 for bin 16 for hex
	  HAL_UART_Transmit(&huart3, str, 6, 1000);
}


