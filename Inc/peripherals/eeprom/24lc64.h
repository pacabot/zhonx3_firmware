/**************************************************************************/
/*!
    @file     24LC64.h
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Driver for eeprom 24LC64
 */
/**************************************************************************/
#ifndef __24LC64_H__
#define __24LC64_H__

#include "stdbool.h"

#define WRITE_PROTECT GPIO_PIN_3

void eepromWP(char state);
void eeepromWriteByte(unsigned int eeaddress, unsigned char data);
void eepromWritePage(unsigned int eeaddresspage, unsigned char* data, unsigned char length);
char eepromReadByte(unsigned int eeaddress);
void eepromReadBuffer(unsigned int eeaddress, unsigned char *buffer, int length);
void eepromTest(void);

#endif
