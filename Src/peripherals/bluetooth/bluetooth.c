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

// Array of possible baud rates
const int baudrates[] =
{
		1200,
		2400,
		4800,
		9600,
		19200,
		38400,
		57600,
		115200,
		230400,
		460800,
		//921600,
		-0x7FFFFFFF    // Indicates the end of the array
};

// This variable is used to change the baudrate
int BTBaudrate = 115200;

presetParam BTpresetBaudRate =
{
		(void *)&BTBaudrate,
		(void *)baudrates,
		bluetoothSetBaudrate
};

static inline int bluetoothInit_IT(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	return HAL_OK;
}

static inline int bluetoothDeInit_IT(void)
{
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);

	// Pause DMA transfer on UART3
	//    HAL_UART_DMAPause(&huart3);

	huart3.gState = HAL_UART_STATE_READY;

	return HAL_OK;
}

void bluetoothInit(void)
{
	char *resp;
	unsigned char c;
	int rv;

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306Printf(12, 30, &Font_3x6, "%s", "Initializing Bluetooth...");
	ssd1306Refresh();

	// Allow Remote Escape Sequence
	bluetoothCmd("AT+AB Config RmtEscapeSequence = true");

	// TODO: Check if this delay is necessary
	//HAL_Delay(100);

	// Reset the bluetooth peripheral
	bluetoothCmd("AT+AB Reset");

	// Initialize USART Interrupts
	bluetoothInit_IT();
}

int bluetoothSend(unsigned char *data, int length)
{
	return HAL_UART_Transmit_DMA(&huart3, data, length);
}

int bluetoothReceive(unsigned char *data, int length)
{
	return HAL_UART_Receive_DMA(&huart3, data, length);
}

void bluetoothPrintf(const char *format, ...)
{
	static char buffer[BLUETOOTH_BUFFER_SIZE];
	va_list va_args;

	va_start(va_args, format);
	vsnprintf(buffer, BLUETOOTH_BUFFER_SIZE, format, va_args);
	va_end(va_args);

	bluetoothSend((unsigned char *)buffer, strlen(buffer));
}

void bluetoothWaitReady(void)
{
	// Wait until UART becomes ready
	while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY);
}

char *bluetoothCmd(const char *cmd)
{
	HAL_StatusTypeDef rv;
	static char response[255];
	char command[50];
	char *p_response = response;

	memset(response, 0, sizeof(response));

	// Disable RXNE interrupts
	bluetoothDeInit_IT();

	strcpy(command, cmd);
	strcat(command, "\r\n");

	HAL_UART_Transmit(&huart3, command, strlen(command), 5000);

	// Wait until end of reception
	do
	{
		rv = HAL_UART_Receive(&huart3, p_response++, 1, 200);
	}
	while (rv != HAL_TIMEOUT);

	// Put a NULL character at the end of the response string
	*p_response = '\0';

	bluetoothInit_IT();

	return response;
}

int bluetoothSetBaudrate(int baudrate, void *param)
{
	int rv;
	char cmd[40];
	char *response;

	UNUSED(param);

	// Send command to Bluetooth module
	sprintf(cmd, "AT+AB ChangeBaud %i", baudrate);
	response = bluetoothCmd(cmd);

	// Set baudrate of CPU USART
	__HAL_UART_DISABLE(&huart3);

	huart3.Init.BaudRate = baudrate;
	//    huart3.Init.WordLength = UART_WORDLENGTH_8B;
	//    huart3.Init.StopBits = UART_STOPBITS_1;
	//    huart3.Init.Parity = UART_PARITY_NONE;
	//    huart3.Init.Mode = UART_MODE_TX_RX;
	//    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//    huart3.Init.OverSampling = UART_OVERSAMPLING_8;
	rv = HAL_UART_Init(&huart3);

	__HAL_UART_ENABLE(&huart3);

	// TODO: Check whether the following code is required
	//bluetoothInit_IT();

	if (rv == HAL_OK)
	{
		return BLUETOOTH_DRIVER_E_SUCCESS;
	}
	return rv;
}


int isBluetoothEvent(char *evnt)
{
	if (strncmp(evnt, "AT-AB ", 6) == 0)
	{
		return TRUE;
	}
	return FALSE;
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
		ssd1306ClearScreen(MAIN_AREA);
		ssd1306DrawString(10, 5, "send hello ZHONX III", &Font_5x8);
		ssd1306PrintInt(10, 15, "nb send = ", i, &Font_5x8);
		ssd1306Refresh();
		i++;
	}
	antiBounceJoystick();
}
