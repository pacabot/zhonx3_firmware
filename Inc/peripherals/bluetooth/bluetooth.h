/**************************************************************************/
/*!
    @file     Bluetooth.h
    @author   PLF Pacabot.com
    @date     03 August 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define BLUETOOTH_DRIVER_E_SUCCESS  0
#define BLUETOOTH_DRIVER_E_ERROR    MAKE_ERROR(BLUETOOTH_DRIVER_MODULE_ID, 1)

/**
 * @brief Initializes Bluetooth peripheral
 *
 * @param 	none
 *
 * @return 	none
 */
void bluetoothInit(void);

/**
 * @brief Prints a formatted string on Bluetooth peripheral
 *
 * This function outputs a formatted string on Bluetooth like printf would do.
 *
 * @param format formatted string to output
 *
 * @return none
 */
void bluetoothPrintf(const char *format, ...);

/**
 * @brief Sends a buffer on Bluetooth peripheral
 *
 * @param data		The buffer to be sent
 * @param length	Length of data parameter
 *
 * @return HAL status
 */
int bluetoothSend(unsigned char *data, int length);

/**
 * @brief Receives data from Bluetooth peripheral
 *
 * @param 	data	Valid pointer toward an external buffer which will contain
 * 					the received data
 * @param   length  Required length
 *
 * @return			Received data length (in bytes) if successful
 * 					BLUETOOTH_DRIVER_E_ERROR otherwise
 */
int bluetoothReceive(unsigned char *data, int length);


/**
 * @brief Send an AT command to Bluetooth module
 *
 * @param cmd       The command to send
 *
 * @return          The response of bluetooth module
 */
char *bluetoothCmd(const char *cmd);


void bluetoothWaitReady(void);

int bluetoothSetBaudrate(int baudrate, void *param);

/*****************************************************************************
 * TEST FUNCTIONS
 *****************************************************************************/
void bluetoothTest(void);

#endif
