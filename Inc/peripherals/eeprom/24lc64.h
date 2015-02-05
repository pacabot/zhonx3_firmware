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

void Eeprom_WP(char state);
void Eeeprom_Write_Byte(unsigned int eeaddress, unsigned char data);
void Eeprom_Write_Page(unsigned int eeaddresspage, unsigned char* data, unsigned char length);
char Eeprom_Read_Byte(unsigned int eeaddress);
void Eeprom_Read_Buffer(unsigned int eeaddress, unsigned char *buffer, int length);
void Debug_Eeprom(void);

#endif
