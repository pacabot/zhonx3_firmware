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
// TODO: Remove middleware declarations, as we are in lower level here
#include "middleware/ring_buffer/ring_buffer.h"
#include "middleware/display/banner.h"

/* Declarations for this module */
#include "peripherals/bluetooth/bluetooth.h"

#define BLUETOOTH_BUFFER_SIZE 512

// Array of possible baud rates
const int baudrates[] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800,
//921600,
-0x7FFFFFFF    // Indicates the end of the array
        };

// This variable is used to change the baudrate
int BTBaudrate = 115200;

presetParam BTpresetBaudRate = { (void *) &BTBaudrate, (void *) baudrates, bluetoothSetBaudrate };

static inline int bluetoothInit_IT(void)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

    return HAL_OK;
}

static inline int bluetoothDeInit_IT(void)
{
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);

    huart3.gState = HAL_UART_STATE_READY;

    return HAL_OK;
}

void bluetoothInit(void)
{
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Printf(12, 30, &Font_3x6, "%s", "Initializing Bluetooth...");
    ssd1306Refresh();

    // Allow Remote Escape Sequence
    bluetoothCmd("AT+AB Config RmtEscapeSequence = true");

    // Reset the bluetooth peripheral
    bluetoothCmd("AT+AB Reset");

    // Enable USART Interrupts
    bluetoothEnable();
}

void bluetoothEnable(void)
{
    // Enable bluetooth Interrupts
    bluetoothInit_IT();

    // Display bluetooth icon on screen
    bannerSetIcon(BLUETOOTH, TRUE);
}

void bluetoothDisable(void)
{
    // Disable bluetooth interrupts
    bluetoothDeInit_IT();

    // Remove bluetooth icone on screen
    bannerSetIcon(BLUETOOTH, FALSE);
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
#if !defined DISABLE_BLUETOOTH
    static char buffer[BLUETOOTH_BUFFER_SIZE];
    va_list va_args;

    va_start(va_args, format);
    vsnprintf(buffer, BLUETOOTH_BUFFER_SIZE, format, va_args);
    va_end(va_args);

    bluetoothSend((unsigned char *) buffer, strlen(buffer));
#endif
}

void bluetoothWaitReady(void)
{
#if !defined DISABLE_BLUETOOTH
    // Wait until UART becomes ready
    while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY);
#endif
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

    HAL_UART_Transmit(&huart3, (uint8_t *)command, strlen(command), 5000);

    // Wait until end of reception
    do
    {
        rv = HAL_UART_Receive(&huart3, (uint8_t *)p_response++, 1, 200);
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

    UNUSED(param);

    // Send command to Bluetooth module
    sprintf(cmd, "AT+AB ChangeBaud %i", baudrate);
    bluetoothCmd(cmd);

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
    while (expanderJoyFiltered() != JOY_LEFT)
    {
        bluetoothPrintf("hello ZHONX_III, nb send = %d\r\n", i);
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(0, 0, "send hello ZHONX III", &Font_5x8);
        ssd1306PrintIntAtLine(0, 1, "nb send = ", i, &Font_5x8);
        ssd1306Refresh();
        i++;
    }
    antiBounceJoystick();
}
