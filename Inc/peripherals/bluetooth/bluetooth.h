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
#define BLUETOOTH_DRIVER_MODULE_ID  1

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
 * @retval HAL status
 */
int bluetoothPrintf(const char *format, ...);

/**
 * @brief Sends a buffer on Bluetooth peripheral
 *
 * @param data		The buffer to be sent
 * @param length	Length of data parameter
 *
 * @retval HAL status
 */
int bluetoothSend(unsigned char *data, int length);

/**
 * @brief Receives data from Bluetooth peripheral
 *
 * @param 	data	Valid pointer toward an external buffer which will contain$$
 * 					the received data
 *
 * @retval			Received data length (in bytes) if successful
 * 					BLUETOOTH_DRIVER_E_ERROR otherwise
 */
int bluetoothReceive(unsigned char *data);

void bluetoothPrintString(const char *text);
void bluetoothPrintInt( const char *text, unsigned int val);
void bluetoothNewLine(void);
void bluetoothTest(void);

#endif
