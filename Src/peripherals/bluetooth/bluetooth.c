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
#include "usart.h"

/* Middleware declarations */
#include "middleware/ring_buffer/ring_buffer.h"

/* Declarations for this module */
#include "peripherals/bluetooth/bluetooth.h"

#define BLUETOOTH_BUFFER_SIZE 512


int bluetoothSend(unsigned char *data, int length)
{
    // TODO: Use DMA to transmit data
	return HAL_UART_Transmit(&huart3, data, length, 1000);
}

int bluetoothReceive(unsigned char *data, int length)
{
    // TODO: Use DMA to receive data
	return HAL_UART_Receive(&huart3, data, length, 1000);
}

void bluetoothPrintf(const char *format, ...)
{
    char buffer[BLUETOOTH_BUFFER_SIZE];
    va_list va_args;

    va_start(va_args, format);
    vsnprintf(buffer, BLUETOOTH_BUFFER_SIZE, format, va_args);
    va_end(va_args);

    bluetoothSend((unsigned char *)buffer, strlen(buffer));
}


/*****************************************************************************
 * TEST FUNCTIONS
 *****************************************************************************/

void bluetoothTest(void)
{
	int i = 0;
	while(expanderJoyFiltered()!=JOY_LEFT)
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
