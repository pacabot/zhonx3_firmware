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

void bluetoothInit(void);
void bluetoothPrintString(const char *text);
void bluetoothPrintInt( const char *text, unsigned int val);
void bluetoothNewLine(void);
void bluetoothTest(void);

#endif
