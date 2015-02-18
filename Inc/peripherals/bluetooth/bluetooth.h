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

void bluetoothInit(void);
void bluetoothPrintString(const char *text);
void bluetoothPrintInt( const char *text, unsigned int val);
void bluetoothNewLine(void);
void bluetoothTest(void);

#endif
