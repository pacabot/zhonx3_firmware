/**************************************************************************/
/*!
    @file    Bluetooth.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/bluetooth/bluetooth.h"

extern UART_HandleTypeDef huart3;

void bluetoothPrintf(const char *format, ...)
{
	char buffer[1024];
	va_list va_args;

	va_start(va_args, format);
    vsnprintf(buffer, 1024, format, va_args);
    va_end(va_args);

    HAL_UART_Transmit(&huart3, (unsigned char *)buffer, strlen(buffer), 1000);
}

void bluetoothPrintString(const char *text)
{
	unsigned char nb_char;

	nb_char = strlen((const char *) text);
	HAL_UART_Transmit(&huart3, (unsigned char*)text, nb_char, 1000);
}

void bluetoothPrintInt( const char *text, unsigned int val)
{
	char str[10];

	sprintf(str, "%d      ", val);
	bluetoothPrintString(text);
	HAL_UART_Transmit(&huart3, (unsigned char*)str, 10, 1000);
}

void bluetoothNewLine(void)
{
	HAL_UART_Transmit(&huart3, (unsigned char*) "\r\n", 2, 1000);
}

void bluetoothTest(void)
{
	int i = 0;
	while(expanderJoyState()!=LEFT)
	{
		bluetoothPrintf("hello ZHONX_III, nb send = %d\r\n", i);
		ssd1306ClearScreen();
		ssd1306DrawString(10, 5, "send hello ZHONX III", &Font_5x8);
		ssd1306PrintInt(10, 15, "nb send = ", i, &Font_5x8);
		ssd1306Refresh();
		i++;
	}
	antiBounceJoystick();
}
